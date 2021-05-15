^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wifi_ddwrt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2021-05-15)
------------------
* Merge pull request `#15 <https://github.com/ros-drivers/wifi_ddwrt/issues/15>`_ from k-okada/add_travis
  add travsi CI
* update CHANGELOG.rst
* Merge pull request `#14 <https://github.com/ros-drivers/wifi_ddwrt/issues/14>`_ from PR2-prime/noetic-devel
* add travsi CI
* updated for focal/noetic/python3 compatibility
* Merge pull request `#7 <https://github.com/ros-drivers/wifi_ddwrt/issues/7>`_ from furushchev/fix-typo
  [wifi_ddwrt] fix typo
* [wifi_ddwrt] fix typo
* Update README.md
* Create README. Fixes `#1 <https://github.com/ros-drivers/wifi_ddwrt/issues/1>`_
* Contributors: Austin, Dave Feil-Seifer, Kei Okada, Yuki Furuta

* updated for focal/noetic/python3 compatibility (`#14 <https://github.com/ros-drivers/wifi_ddwrt/issues/14>`_)
* [wifi_ddwrt] fix typo (`#7 <https://github.com/ros-drivers/wifi_ddwrt/issues/7>`_)
* Create README. Fixes `#1 <https://github.com/ros-drivers/wifi_ddwrt/issues/1>`_
* Contributors: Austin Hendrix, Dave Feil-Seifer, Yuki Furuta

0.2.2 (2021-05-15)
------------------
* 0.2.1
* update changelogs
* Merge pull request `#15 <https://github.com/ros-drivers/wifi_ddwrt/issues/15>`_ from k-okada/add_travis
  add travsi CI
* update CHANGELOG.rst
* Merge pull request `#14 <https://github.com/ros-drivers/wifi_ddwrt/issues/14>`_ from PR2-prime/noetic-devel
* add travsi CI
* updated for focal/noetic/python3 compatibility
* Merge pull request `#7 <https://github.com/ros-drivers/wifi_ddwrt/issues/7>`_ from furushchev/fix-typo
  [wifi_ddwrt] fix typo
* [wifi_ddwrt] fix typo
* Update README.md
* Create README. Fixes `#1 <https://github.com/ros-drivers/wifi_ddwrt/issues/1>`_
* Contributors: Austin, Dave Feil-Seifer, Kei Okada, Yuki Furuta

0.2.0 (2013-06-03)
------------------
* Un-stack.
* Catkinized!
* Remove references to roslib.
* Remove unused options.
* Remove old rosbuild files.
* removing rosdep.yaml
* bumped version for 0.1.5 release
* fix typo, ticket `#5078 <https://github.com/ros-drivers/wifi_ddwrt/issues/5078>`_
* update stack dependencies
* getting ready for 0.1.4 release
* remove deprecated imports
* adding ros_comm depend
* Only print warning when state changes, not every time
* Added Ubuntu platform tags to manifest
* updating to release 0.1.3
* Catch all exception types and make sure we guard fetchSiteSurvey too
* Catch exceptions in fetchCurrentAP, log them, and continue running
* Updating wifi_drivers to 0.1.2
* Took out the gc.collect from each iteration, as it does not seem to be necessary.
* Fixed ticket texas`#135 <https://github.com/ros-drivers/wifi_ddwrt/issues/135>`_ by not reusing the mechanize.Browser, and by manually clearing uncollectable dictionnaries that get left on the heap.
* updating to 0.1.1
* don't try to create an access point object unless we have active data, fixing `#60 <https://github.com/ros-drivers/wifi_ddwrt/issues/60>`_
* wifi_drivers: stack manifest update
* wifi_drivers: Makefile
* wifi_drivers requires python_mechanize rosdep
* preparing wifi_drivers 0.1.0 into tick-tock
* staging wifi_drivers into tick-tock
* wifi_drivers: updating to newer rosbuild commands
* updating to release 0.1.0
* Add wifi_wrt
* Contributors: Austin Hendrix, blaise, gerkey, jamesb, jtyler, kwc, leibs, wheeler, wim
