# Folder: **BIMConnector_xsd**

This folder is linked to [03_Development\Workpackages\Interfaces\XMLFormats\BIMConnector](\\le02fs01.DE.BOSCH.COM\TOP100_BAUTIRO$\internal\03_Development\Workpackages\Interfaces\XMLFormats\BIMConnector)

 1. Version 1  (2022-08-18)
 2. Version 2  (2023-04-04) MaiDemo_2023_Version
 3. Version 3  (2023-06-05)
 4. Version 4  (2023-06-15) additional folders created
 5. Version 5  (2023-11-21) update to TP21 `--> [current]`

## Version 5 (TP21)

1. update Taskplan:  \
   as `TaskPlan-2.1.xsd`  includes - by expecting in same folder - `DataTypes-1.0.xsd` and `NodeTree-2.0.xsd`
2. in comparison to network-share: **MODIFIED**
   1. `NodeTree-2.0.xsd`

      ```xml
      FROM   <xs:include schemaLocation="../../DataTypes-1.0.xsd"/>
      TO     <xs:include schemaLocation="../../DataTypes-1.0/DataTypes-1.0.xsd"/>
      ```

   2.`TaskPlan-2.1.xsd`:

      ```xml
      FROM   <xs:include schemaLocation="DataTypes-1.0.xsd"/>
             <xs:include schemaLocation="NodeTree-2.0.xsd"/>
      TO     <xs:include schemaLocation="../../DataTypes-1.0/DataTypes-1.0.xsd"/>
             <xs:include schemaLocation="../../NodeTree-2.0/NodeTree-2.0.xsd"/>
      ```

```bash
      BimConnector_xsd/
      ├── DataTypes/
      │   └── DataTypes-1.0/
      │       └── DataTypes-1.0.xsd
      ├── NodeTree/
      │   └── NodeTree-2.0/
      │       └── NodeTree-2.0.xsd
      └── TaskPlan/
          ├── TaskPlan-2.0/
          │   ├── TaskPlan-2.0.xsd
          │   └── TaskPlan-2.0  (Le131, Bogen, GPR 1_2_37289, P5).xml
          └── TaskPlan-2.0/
              └── TaskPlan-2.1.xsd

```

## Version 4  (TP20)

file-names and folder-names are kept `preserving the case`,
due to include-statements inside the `*.xsd`-files

```bash
      BimConnector_xsd/
      ├── DataTypes/
      │   └── DataTypes-1.0.xsd
      ├── NodeTree/
      │   └── NodeTree-2.0/
      │       ├── NodeTree-2.0.xsd
      │       └── NodeTree-2.0 (Le131, Bogen, GPR 1_2_37289, P5).xml
      └── TaskPlan/
          └── TaskPlan-2.0/
              ├── TaskPlan-2.0.xsd
              └── TaskPlan-2.0  (Le131, Bogen, GPR 1_2_37289, P5).xml

```
