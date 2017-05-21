# Integration Test with RocketCFS

Verify the building compatibility of GNC models on Flight Software.

## Usage

Run: `~/sirius-simulation $ ./integration_test/integration_test.sh`

## Add New Class

Edit: `~/sirius-simulation $ vim ./integration_test/ins_class.cpp`

Ensure the default constructor of the class is defined.

```
#include "Ins.hh"
Ins ins;
```

