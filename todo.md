Updates
-------

* Update all GUIs to Qt5.

  * Upgrading to Qt5 should fix numerous issues on OS X (including improved retina support, etc).

* Update Python callbacks to Python 3.

* Move the xml scene format to a more human readable format.

* Expanded automated test coverage.

New Features
------------

* User documentation.

* Faceted friction disc support (currently broken).

* Expand the supported collision primitives for 2D and 3D rigid body simulations.

* Expose all functionality through Python bindings.

* Support for mutli-precision floating point operations.

* Support for analytical solutions to rigid body precession.

Performance Improvements
------------------------

* Significant speedups are possible for broad phase collision detection. In the examples that we have tested, broad phase detection is not the bottleneck, but if the speed of broad phase is important in your applications please get in touch.