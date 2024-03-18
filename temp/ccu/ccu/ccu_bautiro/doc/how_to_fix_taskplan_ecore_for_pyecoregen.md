# How to fix TaskPlan.ecore for pyecoregen successful code-generation

## Assume there would be no problem

```bash
pip install pyecore
# pip install pyecoregen

# this here works perfect
pyecoregen -v -e bautiro.ecore   -o ./

# TaskPlan.ecore must be fixed first!
pyecoregen -v -e TaskPlan.ecore  -o ./
´´´

> before that : fix TaskPlan.eCore

´´´text
replace:   2003/XMLType#//  --> 2002/Ecore#//
```

## Background

the file `XMLType.xsd`   \
located here:
`org.eclipse.emf/plugins/org.eclipse.emf.ecore/model/XMLType.xsd`  \
clone it it from here:
`https://git.eclipse.org/r/emf/org.eclipse.emf.git`  \

Contains a mapping from XML-Schema Simple Type  \
to ECore (and Java) Simple Type

*e.g.*

```xml
eType="EDataType http://www.eclipse.org/emf/2003/XMLType#//String"
eType="EDataType http://www.eclipse.org/emf/2002/Ecore#//String"
```

While the eclipse code generator works out of the BOX -

pyecoregen fails loading the XMLType

```bash
Resource "http://www.eclipse.org/emf/2003/XMLType" cannot be resolved
```

Quick fix: the types originally defined in `Taskplan-1.5.XSD`,

Namely:

 | XML-XSD-Type                     | ECORE-Type               | Comment                                              |
 | -------------------------------- | ------------------------ | ---------------------------------------------------- |
 | emf/2003/XMLType#//Double        | emf/2002/Ecore#//EDouble | EDouble is java double is 64 bits (10^324 .. 10^308) |
 | emf/2003/XMLType#//String        | emf/2002/Ecore#//EString |                                                      |
 | emf/2003/XMLType#//NCName        | emf/2002/Ecore#//EString | None-Colonized String (c-code variables)             |
 | emf/2003/XMLType#//DateTime      | emf/2002/Ecore#//EString | DateTime According to xsd                            |
 | emf/2003/XMLType#//Duration      | emf/2002/Ecore#//EString | Duration According to xsd                            |
 | emf/2003/XMLType#//AnySimpleType | emf/2002/Ecore#//EString | can be anything (integer, float, string)             |

## Full Mapping Table

can be found in

> EMF: Eclipse Modeling Framework (by Ed.Merks)
>
> Section9.9. Predefined Schema Simple Types (Page 223)

### nächstes Problem

    AttributeError: 'NoneType' object has no attribute 'eClass'

? keine Ahnung was die Ursache ist

### Try out

ersetze die 5 simpleTypes durch String.

d.h.
entferne diese 5 Definitionen.

```xml
<eClassifiers xsi:type="ecore:EDataType" name="CodeType" instanceClassName="java.lang.String"></eClassifiers>
<eClassifiers xsi:type="ecore:EDataType" name="GuidType" instanceClassName="java.lang.String"></eClassifiers>
<eClassifiers xsi:type="ecore:EDataType" name="MaterialType" instanceClassName="java.lang.String"></eClassifiers>
<eClassifiers xsi:type="ecore:EDataType" name="StateType" instanceClassName="java.lang.String"></eClassifiers>
<eClassifiers xsi:type="ecore:EDataType" name="ToolType" instanceClassName="java.lang.String"></eClassifiers>
```

und ersetze

```python
eType="#//CodeType"
eType="#//GuidType"
eType="#//MaterialType"
eType="#//StateType"
eType="#//ToolType"

eType="ecore:EDataType http://www.eclipse.org/emf/2002/Ecore#//EString"

```
