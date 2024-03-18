# ccu_bautiro (Python Package)

> **``! GENERATED CODE !``**
>
> This python package is generated code
>
> **``DON'T DO MANUAL CHANGES``**

## How to code-gen

```bash
./do_codegen.bash
```

pyecoregen consumes the *META*-model (`model/ccu_bautiro.ecore`)
and generates static `Python` classes from it.

## Dependencies

see [setup.py](setup.py).

## Example Usage in Python

```python
from ccu_bautiro.NodeTree import CNode

def create_node_tree() -> List[CNode]:

   n = CNode()
   n.name = "Ich_bin_ein_Node"

   c = CNode()
   c.name = "Ich_bin_ein_kind_Node"

   n.children.append(c)

   return list(n)
```
