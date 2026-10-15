use std::collections::HashMap;

/// Minimal JSON value tree, sufficient for FDM configuration payloads.
#[derive(Clone, Debug, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<Json>),
    Obj(Vec<(String, Json)>),
}

impl Json {
    pub fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(fields) => fields.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }
    pub fn num(&self) -> Option<f64> {
        match self {
            Json::Num(n) => Some(*n),
            _ => None,
        }
    }
    pub fn arr(&self) -> Option<&[Json]> {
        match self {
            Json::Arr(a) => Some(a),
            _ => None,
        }
    }
    pub fn bool(&self) -> Option<bool> {
        match self {
            Json::Bool(b) => Some(*b),
            _ => None,
        }
    }
    pub fn str(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s),
            _ => None,
        }
    }
}

/// Recursive-descent JSON parser over a byte slice.
pub struct Parser<'a> {
    b: &'a [u8],
    i: usize,
}

impl<'a> Parser<'a> {
    pub fn new(b: &'a [u8]) -> Parser<'a> {
        Parser { b, i: 0 }
    }

    pub fn parse(&mut self) -> Result<Json, ()> {
        self.skip_ws();
        let v = self.value()?;
        self.skip_ws();
        if self.i != self.b.len() {
            return Err(());
        }
        Ok(v)
    }

    fn skip_ws(&mut self) {
        while self.i < self.b.len() && matches!(self.b[self.i], b' ' | b'\t' | b'\n' | b'\r') {
            self.i += 1;
        }
    }

    fn peek(&self) -> Result<u8, ()> {
        self.b.get(self.i).copied().ok_or(())
    }

    fn eat(&mut self, c: u8) -> Result<(), ()> {
        if self.peek()? == c {
            self.i += 1;
            Ok(())
        } else {
            Err(())
        }
    }

    fn value(&mut self) -> Result<Json, ()> {
        self.skip_ws();
        match self.peek()? {
            b'{' => self.object(),
            b'[' => self.array(),
            b'"' => Ok(Json::Str(self.string()?)),
            b't' => self.keyword("true", Json::Bool(true)),
            b'f' => self.keyword("false", Json::Bool(false)),
            b'n' => self.keyword("null", Json::Null),
            _ => Ok(Json::Num(self.number()?)),
        }
    }

    fn object(&mut self) -> Result<Json, ()> {
        self.eat(b'{')?;
        let mut fields = Vec::new();
        self.skip_ws();
        if self.peek()? == b'}' {
            self.i += 1;
            return Ok(Json::Obj(fields));
        }
        loop {
            self.skip_ws();
            let key = self.string()?;
            self.skip_ws();
            self.eat(b':')?;
            let val = self.value()?;
            fields.push((key, val));
            self.skip_ws();
            match self.peek()? {
                b',' => {
                    self.i += 1;
                }
                b'}' => {
                    self.i += 1;
                    break;
                }
                _ => return Err(()),
            }
        }
        Ok(Json::Obj(fields))
    }

    fn array(&mut self) -> Result<Json, ()> {
        self.eat(b'[')?;
        let mut items = Vec::new();
        self.skip_ws();
        if self.peek()? == b']' {
            self.i += 1;
            return Ok(Json::Arr(items));
        }
        loop {
            self.skip_ws();
            items.push(self.value()?);
            self.skip_ws();
            match self.peek()? {
                b',' => {
                    self.i += 1;
                }
                b']' => {
                    self.i += 1;
                    break;
                }
                _ => return Err(()),
            }
        }
        Ok(Json::Arr(items))
    }

    fn keyword(&mut self, word: &str, value: Json) -> Result<Json, ()> {
        if self.b.len() - self.i < word.len() || &self.b[self.i..self.i + word.len()] != word.as_bytes() {
            return Err(());
        }
        self.i += word.len();
        Ok(value)
    }

    fn string(&mut self) -> Result<String, ()> {
        self.eat(b'"')?;
        let mut out = Vec::new();
        loop {
            let c = self.peek()?;
            self.i += 1;
            match c {
                b'"' => break,
                b'\\' => {
                    let e = self.peek()?;
                    self.i += 1;
                    match e {
                        b'n' => out.push(b'\n'),
                        b't' => out.push(b'\t'),
                        b'r' => out.push(b'\r'),
                        b'b' => out.push(0x08),
                        b'f' => out.push(0x0c),
                        b'u' => {
                            let hi = self.hex4()?;
                            let mut cp = hi as u32;
                            if (0xd800..0xdc00).contains(&hi) {
                                if self.peek()? == b'\\' {
                                    self.i += 1;
                                    if self.peek()? == b'u' {
                                        self.i += 1;
                                        let lo = self.hex4()?;
                                        cp = 0x10000 + ((hi as u32 - 0xd800) << 10) + (lo as u32 - 0xdc00);
                                    }
                                }
                            }
                            out.extend_from_slice(&char::from_u32(cp).ok_or(())?.to_string().into_bytes());
                        }
                        other => out.push(other),
                    }
                }
                c if c < 0x20 => return Err(()),
                c => out.push(c),
            }
        }
        String::from_utf8(out).map_err(|_| ())
    }

    fn hex4(&mut self) -> Result<u16, ()> {
        if self.i + 4 > self.b.len() {
            return Err(());
        }
        let mut v: u16 = 0;
        for _ in 0..4 {
            let c = self.b[self.i];
            self.i += 1;
            let d = match c {
                b'0'..=b'9' => c - b'0',
                b'a'..=b'f' => c - b'a' + 10,
                b'A'..=b'F' => c - b'A' + 10,
                _ => return Err(()),
            };
            v = v * 16 + d as u16;
        }
        Ok(v)
    }

    fn number(&mut self) -> Result<f64, ()> {
        let start = self.i;
        if self.peek()? == b'-' {
            self.i += 1;
        }
        while self.i < self.b.len() && matches!(self.b[self.i], b'0'..=b'9') {
            self.i += 1;
        }
        if self.i < self.b.len() && self.b[self.i] == b'.' {
            self.i += 1;
            while self.i < self.b.len() && matches!(self.b[self.i], b'0'..=b'9') {
                self.i += 1;
            }
        }
        if self.i < self.b.len() && matches!(self.b[self.i], b'e' | b'E') {
            self.i += 1;
            if self.i < self.b.len() && matches!(self.b[self.i], b'+' | b'-') {
                self.i += 1;
            }
            while self.i < self.b.len() && matches!(self.b[self.i], b'0'..=b'9') {
                self.i += 1;
            }
        }
        if self.i == start {
            return Err(());
        }
        std::str::from_utf8(&self.b[start..self.i]).ok().and_then(|s| s.parse().ok()).ok_or(())
    }
}

pub fn parse(bytes: &[u8]) -> Result<Json, ()> {
    Parser::new(bytes).parse()
}

/// Flatten a JSON object's immediate numeric fields into a `HashMap`.
pub fn obj_nums(json: &Json) -> HashMap<String, f64> {
    let mut out = HashMap::new();
    if let Json::Obj(fields) = json {
        for (k, v) in fields {
            if let Json::Num(n) = v {
                out.insert(k.clone(), *n);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_nested_object() {
        let j = parse(br#"{"mass":1.4,"inertia":[0.019,0.019,0.0252],"motor":{"kv":960},"ok":true,"none":null,"s":"x"}"#).unwrap();
        assert_eq!(j.get("mass").unwrap().num(), Some(1.4));
        assert_eq!(j.get("inertia").unwrap().arr().unwrap().len(), 3);
        assert_eq!(j.get("motor").unwrap().get("kv").unwrap().num(), Some(960.0));
        assert_eq!(j.get("ok").unwrap().bool(), Some(true));
        assert_eq!(j.get("none").unwrap(), &Json::Null);
        assert_eq!(j.get("s").unwrap().str(), Some("x"));
    }

    #[test]
    fn parses_negatives_and_exponents() {
        let j = parse(br#"[-1.5,6.05e-5,2E3]"#).unwrap();
        let a = j.arr().unwrap();
        assert!((a[0].num().unwrap() + 1.5).abs() < 1e-12);
        assert!((a[1].num().unwrap() - 6.05e-5).abs() < 1e-12);
        assert_eq!(a[2].num(), Some(2000.0));
    }

    #[test]
    fn rejects_garbage() {
        assert!(parse(b"{\"a\":}").is_err());
        assert!(parse(b"[1,2,]").is_err());
        assert!(parse(b"abc").is_err());
        assert!(parse(b"{\"a\":1}x").is_err());
    }

    #[test]
    fn handles_escapes() {
        let j = parse(br#""a\n\t\u00e9""#).unwrap();
        assert_eq!(j.str(), Some("a\n\t\u{e9}"));
    }
}