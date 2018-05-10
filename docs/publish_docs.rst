How to build and publish documentation
**************************************

Documentation can be found at http://people.ciirc.cvut.cz/tesarm11/pepper_docs/

There are two scripts for this purpose:

1. Building the documentation
2. Publishing it to server

Building
========

For building it is used `sphinx-docs` framework with `rST` docstring formatting.

To build a documentation only run `build_docs.sh` script which generates
a output to `_build/` folder.

If everything is looking good it can be easysily published to a server.

Publishing
==========

For publishing only run `publish_docs.sh` in the root of the repository.
