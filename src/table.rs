pub fn interp(table: &[(f64, f64)], x: f64) -> f64 {
    let n = table.len();
    if x <= table[0].0 {
        return table[0].1;
    }
    if x >= table[n - 1].0 {
        return table[n - 1].1;
    }
    let mut lo = 0;
    let mut hi = n - 1;
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if table[mid].0 <= x {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let (x0, y0) = table[lo];
    let (x1, y1) = table[hi];
    y0 + (y1 - y0) * (x - x0) / (x1 - x0)
}
