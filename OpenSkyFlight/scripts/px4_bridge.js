#!/usr/bin/env node

// PX4 SITL bridge: relays MAVLink bytes between PX4 (TCP client) and the
// browser FDM (WebSocket). PX4 connects to --tcp-port (default 4560); the page
// connects to ws://localhost:<ws-port> (default 3001).
//
// Usage: node scripts/px4_bridge.js [--tcp-port 4560] [--ws-port 3001]

import { createServer as createTcpServer } from 'node:net';
import { createServer as createHttpServer } from 'node:http';
import { createHash } from 'node:crypto';
import { parseArgs } from 'node:util';

const { values } = parseArgs({
  options: {
    'tcp-port': { type: 'string', default: '4560' },
    'ws-port': { type: 'string', default: '3001' },
  },
});

const GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';

class WsConn {
  constructor(socket) {
    this.socket = socket;
    this.buf = Buffer.alloc(0);
    this.onmessage = null;
    this.onclose = null;
  }

  onData(chunk) {
    this.buf = Buffer.concat([this.buf, chunk]);
    this._parse();
  }

  _parse() {
    for (;;) {
      if (this.buf.length < 2) return;
      const b0 = this.buf[0];
      const b1 = this.buf[1];
      const opcode = b0 & 0x0f;
      const masked = (b1 & 0x80) !== 0;
      let len = b1 & 0x7f;
      let off = 2;
      if (len === 126) {
        if (this.buf.length < 4) return;
        len = this.buf.readUInt16BE(2);
        off = 4;
      } else if (len === 127) {
        if (this.buf.length < 10) return;
        len = Number(this.buf.readBigUInt64BE(2));
        off = 10;
      }
      let mask = null;
      if (masked) {
        if (this.buf.length < off + 4) return;
        mask = this.buf.subarray(off, off + 4);
        off += 4;
      }
      if (this.buf.length < off + len) return;
      let payload = this.buf.subarray(off, off + len);
      if (masked) {
        const out = Buffer.alloc(len);
        for (let i = 0; i < len; i++) out[i] = payload[i] ^ mask[i % 4];
        payload = out;
      } else {
        payload = Buffer.from(payload);
      }
      this.buf = this.buf.subarray(off + len);

      if (opcode === 0x8) {
        this.close();
        return;
      } else if (opcode === 0x9) {
        this._frame(0xa, payload);
      } else if (opcode === 0x1 || opcode === 0x2 || opcode === 0x0) {
        if (this.onmessage) this.onmessage(payload, opcode);
      }
    }
  }

  _frame(opcode, payload) {
    const len = payload.length;
    let header;
    if (len < 126) {
      header = Buffer.from([0x80 | opcode, len]);
    } else if (len < 65536) {
      header = Buffer.alloc(4);
      header[0] = 0x80 | opcode;
      header[1] = 126;
      header.writeUInt16BE(len, 2);
    } else {
      header = Buffer.alloc(10);
      header[0] = 0x80 | opcode;
      header[1] = 127;
      header.writeBigUInt64BE(BigInt(len), 2);
    }
    try {
      this.socket.write(Buffer.concat([header, payload]));
    } catch {
      /* ignore */
    }
  }

  sendBinary(buf) {
    this._frame(0x2, buf);
  }

  sendText(s) {
    this._frame(0x1, Buffer.from(s));
  }

  close() {
    try {
      this.socket.end();
    } catch {
      /* ignore */
    }
    if (this.onclose) this.onclose();
  }
}

let px4 = null;
let browser = null;
let intervalPx4ToBrowser = 0;
let intervalBrowserToPx4 = 0;

const broadcast = (text) => {
  if (browser) browser.sendText(text);
};

const tcp = createTcpServer((sock) => {
  if (px4) {
    try {
      px4.destroy();
    } catch {
      /* ignore */
    }
  }
  px4 = sock;
  sock.setNoDelay(true);
  console.log('PX4 TCP connected');
  broadcast('PX4:1');
  sock.on('data', (d) => {
    if (browser) {
      intervalPx4ToBrowser += d.length;
      browser.sendBinary(d);
    }
  });
  sock.on('close', () => {
    if (px4 === sock) px4 = null;
    console.log('PX4 TCP disconnected');
    broadcast('PX4:0');
  });
  sock.on('error', () => {});
});

const http = createHttpServer((req, res) => {
  res.writeHead(200, { 'Content-Type': 'text/plain' });
  res.end('px4 bridge\n');
});

http.on('upgrade', (req, socket) => {
  const key = req.headers['sec-websocket-key'];
  if (!key) {
    socket.destroy();
    return;
  }
  const accept = createHash('sha1')
    .update(key + GUID)
    .digest('base64');
  socket.write(
    'HTTP/1.1 101 Switching Protocols\r\n' +
      'Upgrade: websocket\r\n' +
      'Connection: Upgrade\r\n' +
      `Sec-WebSocket-Accept: ${accept}\r\n\r\n`,
  );
  socket.setNoDelay(true);

  const conn = new WsConn(socket);
  if (browser) {
    try {
      browser.close();
    } catch {
      /* ignore */
    }
  }
  browser = conn;
  conn.onmessage = (payload) => {
    if (px4) {
      intervalBrowserToPx4 += payload.length;
      px4.write(payload);
    }
  };
  conn.onclose = () => {
    if (browser === conn) browser = null;
  };
  socket.on('data', (c) => conn.onData(c));
  socket.on('close', () => conn.close());
  socket.on('error', () => {});
  conn.sendText(px4 ? 'PX4:1' : 'PX4:0');
  console.log(`Browser WebSocket connected (PX4 ${px4 ? 'connected' : 'disconnected'})`);
});

setInterval(() => {
  console.log(
    `Relay: PX4=${px4 ? 'connected' : 'disconnected'}, browser=${browser ? 'connected' : 'disconnected'}, ` +
      `PX4→browser ${intervalPx4ToBrowser} B / browser→PX4 ${intervalBrowserToPx4} B`,
  );
  intervalPx4ToBrowser = 0;
  intervalBrowserToPx4 = 0;
}, 5000).unref();

tcp.listen(Number(values['tcp-port']), () => {
  console.log(`PX4 bridge: TCP listening on 0.0.0.0:${values['tcp-port']} (PX4 connects here)`);
});
http.listen(Number(values['ws-port']), () => {
  console.log(`PX4 bridge: WebSocket listening on ws://localhost:${values['ws-port']} (browser connects here)`);
});
