# License proposal

## Recommended license: BSD-3-Clause

BSD-3-Clause is recommended for the original EmergentCamera adapter code:

- It permits source and binary redistribution, modification, and commercial
  use while requiring retention of copyright notices and license terms.
- It prevents use of the copyright holder's or contributors' names for
  endorsement without permission.
- It matches the BSD-3-Clause license used by Micro-Manager's MMDevice
  support code and fits Micro-Manager's preference for compatible open-source
  adapter licenses.

This is a proposal only. No license has been assigned to the original adapter
code by this reorganization. The copyright holder (the individual author or
the company, as applicable) must approve the choice and copyright statement
before a final adapter `LICENSE` and SPDX notices are added.

References:

- [OSI BSD-3-Clause license text](https://opensource.org/license/bsd-3-clause)
- [Micro-Manager contribution policy](https://micro-manager.org/Policies_for_inclusion_of_source_code)

## Existing third-party terms

The original adapter code and the Micro-Manager support code are separate
copyrighted works. MMDevice retains its existing BSD-3-Clause terms, reproduced
in [licenses/MMDevice-BSD-3-Clause.txt](../licenses/MMDevice-BSD-3-Clause.txt).
The Linux binary includes compiled MMDevice support code, so this notice is
included with the binaries. Other files in the containing Micro-Manager
repository retain their existing licenses.

The Emergent eSDK is an external vendor dependency. Its headers, libraries,
drivers, and runtime are not included here and remain subject to Emergent's
own licensing terms. The proposed adapter license does not relicense them.
