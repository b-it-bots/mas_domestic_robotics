# Description
<!-- Briefly describe what this MR is about, where the changes occurred-->

## What is being refactored?
<!-- List whatever you have documented, feel free to add things that are not included in this list -->
- [ ] Code
- [ ] Package organization
- [ ] (Other -- please add the options)

# Motivation
<!-- Why was the update necessary -->

# Organization
<!-- If components are being moved, please list them below. You can use the command tree for that -->

## Current

```
.
├── mcr_example
│   ├── CMakeLists.txt
│   └── package.xml
└── mcr_some_package
    ├── CMakeLists.txt
    ├── package.xml
    └── ros
        ├── launch
        │   └── some_package.launch
        └── src
            └── some_package_node.cpp
```
## Proposed
```
.
├── mcr_example
│   ├── CMakeLists.txt
│   └── package.xml
└── mcr_some_package
    ├── CMakeLists.txt
    ├── package.xml
    └── ros
        ├── launch
        │   └── some_package.launch
        └── src
            └── some_package_node.cpp

```
