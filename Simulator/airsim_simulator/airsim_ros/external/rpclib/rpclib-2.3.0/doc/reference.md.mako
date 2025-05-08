<style>
    .table td {
        text-align: left;
    }

    .table td:first-child {
        text-align: right;
        width: 230px;
    }​
</style>
This document is a detailed and (hopefully) complete reference of the public interface of `rpclib`.
For a tutorial, take a look at the [Primer](primer.md). Also, you can find many examples in the [Cookbook](cookbook.md).

<%!
def opt(s):
    if s != None:
        return s
    else:
        return ''
%>

% for c in classes:
${"##"} ${c.name}

```cpp
${"#include "}"${c.includes}"
```
${"### Description"}

${c.brief}

${opt(c.desc)}

${"### Public functions"}

| | |
|---------|-------------|
    % for f in c.functions:
| ${opt(f.type)} | [${f.name}](${"#"}${f.id})${f.argsstr}
    % endfor


    % for f in c.functions:
<h4 id="${f.id}" class="doxy">${c.name}::${f.name}</h4>
```cpp
${opt(f.type)} ${c.name}::${f.name}${f.argsstr};
```

${f.brief}

        % if f.tparams:
${"##### Template parameters"}
            % for t in f.tparams:
`${t.name}` ${t.desc}

            % endfor
        % endif
        % if f.params:
${"##### Parameters"}
            % for p in f.params:
`${p.name}` ${p.desc}

            % endfor
        % endif
        % if f.desc:
${"##### Details"}
${f.desc}

        % endif
        % if f.returns:
${"##### Return value"}
${f.returns}

        % endif
        % if f.note:
!!! warn
    ${f.note}
        % endif

    % endfor

% endfor
