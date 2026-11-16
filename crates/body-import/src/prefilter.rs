//! REQ-012 C02:解析前去掉 `<visual>`(任务书 1.1)。
//! 纯文本扫描,不解析 XML;注释里的内容原样保留。

/// 去掉 URDF 文本里所有 `<visual …>…</visual>` 块和自闭合的 `<visual …/>`。
/// 返回(去掉后的文本,去掉的块数)。
/// - 只认标签名恰好是 `visual` 的元素(`<visual` 后紧跟空白、`>` 或 `/`);
/// - `<visual>` 不嵌套,遇到开标签就找其后第一个 `</visual>`;
/// - 找不到配对的 `</visual>` → Err(文本含“visual 未闭合”);
/// - XML 注释 `<!-- … -->` 里的内容不处理,原样保留。
pub(crate) fn strip_visuals(xml: &str) -> Result<(String, usize), String> {
    let mut out = String::with_capacity(xml.len());
    let mut count = 0usize;
    let mut rest = xml;
    while !rest.is_empty() {
        if rest.starts_with("<!--") {
            let end = rest.find("-->").map(|p| p + 3).unwrap_or(rest.len());
            out.push_str(&rest[..end]);
            rest = &rest[end..];
            continue;
        }
        if let Some(after) = rest.strip_prefix("<visual") {
            let next = after.chars().next();
            let is_visual_tag =
                matches!(next, Some('>') | Some('/')) || next.is_some_and(char::is_whitespace);
            if is_visual_tag {
                let gt = after
                    .find('>')
                    .ok_or_else(|| "visual 未闭合:开标签没有 `>`".to_owned())?;
                if gt > 0 && after.as_bytes()[gt - 1] == b'/' {
                    rest = &after[gt + 1..];
                } else {
                    let body = &after[gt + 1..];
                    let close = body
                        .find("</visual>")
                        .ok_or_else(|| "visual 未闭合:找不到配对的 </visual>".to_owned())?;
                    rest = &body[close + "</visual>".len()..];
                }
                count += 1;
                continue;
            }
        }
        let c = rest.chars().next().expect("rest 非空");
        out.push(c);
        rest = &rest[c.len_utf8()..];
    }
    Ok((out, count))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// P1:两块 + 一个自闭合 → 去掉 3 块;`<visualization>` 原样保留。
    #[test]
    fn p1_strip_blocks_and_self_closing() {
        let xml = "<a><visual><geometry/></visual><b/>\
                   <visual name=\"m\"><material name=\"x\"/></visual>\
                   <c/><visual/><d/>\
                   <visualization>x</visualization></a>";
        let (out, n) = strip_visuals(xml).expect("应成功");
        assert_eq!(n, 3);
        assert_eq!(out, "<a><b/><c/><d/><visualization>x</visualization></a>");
    }

    /// P2:没有闭合 → Err,文本含“visual 未闭合”。
    #[test]
    fn p2_unclosed_is_err() {
        let e = strip_visuals("<a><visual><geometry/></a>").expect_err("应报错");
        assert!(e.contains("visual 未闭合"), "e = {e}");
        let e2 = strip_visuals("<a><visual name=\"m\"").expect_err("开标签无 > 也报错");
        assert!(e2.contains("visual 未闭合"), "e2 = {e2}");
    }

    /// P3:注释里的 `<visual>` 原样保留,不计块数,不报错。
    #[test]
    fn p3_comments_untouched() {
        let xml = "<a><!-- <visual> --><b/><!-- <visual></visual> --></a>";
        let (out, n) = strip_visuals(xml).expect("应成功");
        assert_eq!(n, 0);
        assert_eq!(out, xml);
    }

    /// 标签名须恰好是 visual:`<visuals>`、`<visualization>` 不动。
    #[test]
    fn similar_tag_names_untouched() {
        let xml = "<visuals><visualthing/></visuals><visualization>v</visualization>";
        let (out, n) = strip_visuals(xml).expect("应成功");
        assert_eq!(n, 0);
        assert_eq!(out, xml);
    }

    /// 自闭合带属性、带空白;块前后文本保留。
    #[test]
    fn self_closing_with_attrs() {
        let xml = "前<visual name=\"m\" />后<visual ></visual>尾";
        let (out, n) = strip_visuals(xml).expect("应成功");
        assert_eq!(n, 2);
        assert_eq!(out, "前后尾");
    }
}