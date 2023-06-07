---
tip: translate by baidu@2023-06-07 08:46:30
...

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bondcpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.0 (2022-09-01)

> 4.0.0（2022-09-01）
------------------

* Bond cleanups (`#87 <https://github.com/ros/bond_core/issues/87>`_)

> *债券清理（`#87<https://github.com/ros/bond_core/issues/87>`_)

* Contributors: Michael Carroll

> *撰稿人：Michael Carroll


3.0.2 (2022-04-01)

> 3.0.2（2022-04-01）
------------------


3.0.1 (2021-01-26)

> 3.0.1（2021-01-26）
------------------

* Fix cpplint/uncrustify errors.

> *修复cpplint/unrustify错误。

* Add build dependencies on pkg-config.

> *在pkg配置中添加构建依赖项。

* Contributors: Chris Lalancette

> *撰稿人：Chris Lalancette


3.0.0 (2021-01-26)

> 3.0.0（2021-01-26）
------------------

* [Fixing CI] using chrono literals for durations from rclcpp API update (`#69 <https://github.com/ros/bond_core/issues/69>`_)

> *[Fixing CI]对rclcpp API更新的持续时间使用chrono文字（`#69<https://github.com/ros/bond_core/issues/69>`_)

* Contributors: Steve Macenski

> *撰稿人：Steve Macenski


2.0.0 (2020-11-05)

> 2.0.0（2020年11月5日）
------------------

* Lifecycle support 2 (`#67 <https://github.com/ros/bond_core/issues/67>`_)

> *生命周期支持2（`#67<https://github.com/ros/bond_core/issues/67>`_)

* find uuid correctly on ubuntu and osx (`#55 <https://github.com/ros/bond_core/issues/55>`_)

> *在ubuntu和osx上正确查找uuid（`#55<https://github.com/ros/bond_core/issues/55>`_)

* Ros2 devel (`#54 <https://github.com/ros/bond_core/issues/54>`_)

> *Ros2开发（`#54<https://github.com/ros/bond_core/issues/54>`_)

* Make Michael Carroll the maintainer (`#40 <https://github.com/ros/bond_core/issues/40>`_)

> *让Michael Carroll担任维护者（`#40<https://github.com/ros/bond_core/issues/40>`_)

* Contributors: Karsten Knese, Mikael Arguedas, Steve Macenski

> *撰稿人：Karsten Knese、Mikael Arguedas、Steve Macenski


1.8.3 (2018-08-17)

> 1.8.3（2018-08-17）
------------------

* Argument to Boost Milliseconds must be integral in Boost >= 1.67 (`#37 <https://github.com/ros/bond_core/issues/37>`_)

> *Boost毫秒的参数必须是Boost中的积分>=1.67（`#37<https://github.com/ros/bond_core/issues/37>`_)

  * Argument to Boost milliseconds  must be integral

> *Boost毫秒的参数必须是整数

  * Fix style

> *修复样式

  * More consistent type

> *更一致的类型

* Contributors: Paul-Edouard Sarlin

> *撰稿人：Paul Edouard Sarlin


1.8.2 (2018-04-27)

> 1.8.2（2018-04-27）
------------------

* uuid dependency fixup (`#36 <https://github.com/ros/bond_core/issues/36>`_)

> *uuid依赖性修复程序（`#36<https://github.com/ros/bond_core/issues/36>`_)

  * dont export uuid dependency as this isnt anywhere in the public api

> *不要导出uuid依赖项，因为它不在公共api中的任何位置

  * fixx uuid dependency in test_bond as well

> *修复test_bond中的uuid依赖项

* Contributors: Mikael Arguedas

> *撰稿人：Mikael Arguedas


1.8.1 (2017-10-27)

> 1.8.1（2017-10-27）
------------------

* fix package.xml to comply with schema (`#30 <https://github.com/ros/bond_core/issues/30>`_)

> *修复package.xml以符合架构（`#30<https://github.com/ros/bond_core/issues/30>`_)

* Contributors: Mikael Arguedas

> *撰稿人：Mikael Arguedas


1.8.0 (2017-07-27)

> 1.8.0（2017-07-27）
------------------

* Use SteadyTime and SteadyTimer for bond timeouts (`#18 <https://github.com/ros/bond_core/issues/18>`_)

> *将SteadyTime和SteadyTimer用于绑定超时（`#18<https://github.com/ros/bond_core/issues/18>`_)

* C++ style (`#28 <https://github.com/ros/bond_core/issues/28>`_)

> *C++样式（`#28<https://github.com/ros/bond_core/issues/28>`_)

* switch to package format 2 (`#27 <https://github.com/ros/bond_core/issues/27>`_)

> *切换到包格式2（`#27<https://github.com/ros/bond_core/issues/27>`_)

* remove trailing whitespaces (`#26 <https://github.com/ros/bond_core/issues/26>`_)

> *删除尾部空白（`#26<https://github.com/ros/bond_core/issues/26>`_)

* Contributors: Felix Ruess, Mikael Arguedas

> *撰稿人：Felix Ruess，Mikael Arguedas


1.7.19 (2017-03-27)

> 1.7.19（2017-03-27）
-------------------

* fix unused var warning

> *修复未使用的var警告

* Contributors: Mikael Arguedas

> *撰稿人：Mikael Arguedas


1.7.18 (2016-10-24)

> 1.7.18（2016年10月24日）
-------------------

* fix -isystem /usr/include build breakage in gcc6

> *修复了gcc6中issystem/usr/include构建中断的问题

* Contributors: Mikael Arguedas

> *撰稿人：Mikael Arguedas


1.7.17 (2016-03-15)

> 1.7.17（2016-03-15）
-------------------

* update maintainer

> *更新维护器

* Contributors: Mikael Arguedas

> *撰稿人：Mikael Arguedas


1.7.16 (2014-10-30)

> 1.7.16（2014-10-30）
-------------------

* Fix depedency version

> *修复depedency版本

* Contributors: Esteve Fernandez

> *撰稿人：Esteve Fernandez


1.7.15 (2014-10-28)

> 1.7.15（2014-10-28）
-------------------

* Added version dependency.

> *添加了版本依赖项。

* Removed redundant include_directories

> *删除了冗余include_directories

* Added cmake_modules in alphabetical order

> *按字母顺序添加cmake_modules

* Use FindUUID.cmake from cmake-modules to find the UUID libraries `#8 <https://github.com/ros/bond_core/pull/8>`_

> *使用cmake模块中的FindUUID.cmake查找UUID库`#8<https://github.com/ros/bond_core/pull/8>`_

* Contributors: Esteve Fernandez

> *撰稿人：Esteve Fernandez


1.7.14 (2014-05-08)

> 1.7.14（2014-05-08）
-------------------

* Update maintainer field

> *更新maintainer字段

* Contributors: Esteve Fernandez, Vincent Rabaud

> *撰稿人：Esteve Fernandez，Vincent Rabaud


1.7.13 (2013-08-21)

> 1.7.13（2013-08-21）
-------------------

* Use c++ style reinterpret_cast rather than c style cast

> *使用c++样式的reinterpret_cast，而不是c样式的强制转换

* use rpc for uuid on windows

> *在windows上对uuid使用rpc

* add missing archive/library/runtime destinations for library

> *为库添加缺少的存档/库/运行时目标

* Contributors: David Hodo, Dirk Thomas, William Woodall

> *撰稿人：David Hodo，Dirk Thomas，William Woodall


1.7.12 (2013-06-06)

> 1.7.12（2013-06-06）
-------------------

* fix dependency on exported targets if the variable is empty

> *修复变量为空时对导出目标的依赖关系

* use EXPORTED_TARGETS variable instead of explicit target names

> *使用EXPORTED_TARGETS变量而不是显式目标名称

* Contributors: Dirk Thomas

> *投稿人：Dirk Thomas


1.7.11 (2013-03-13)

> 1.7.11（2013-03-13）
-------------------


1.7.10 (2013-01-13)

> 1.7.10（2013-01-13）
-------------------

* add missing link library uuid `#6 <https://github.com/ros/bond_core/issues/6>`_

> *添加缺少的链接库uuid`#6<https://github.com/ros/bond_core/issues/6>`_

* Contributors: Dirk Thomas

> *投稿人：Dirk Thomas


1.7.9 (2012-12-27)

> 1.7.9（2012-12-27）
------------------

* modified dep type of catkin

> *改良dep型柳絮

* Contributors: Dirk Thomas

> *投稿人：Dirk Thomas


1.7.8 (2012-12-13)

> 1.7.8（2012-12-13）
------------------


1.7.7 (2012-12-06)

> 1.7.7（2012-12-06）
------------------

* Added missing link against catkin_LIBRARIES

> *添加了针对catkin_LIBRARIES的缺失链接

* Updated url tags in package.xml's `#1 <https://github.com/ros/bond_core/pull/1>`_

> *已更新package.xml的`#1中的url标记<https://github.com/ros/bond_core/pull/1>`_

* updated catkin_package(DEPENDS)

> *更新的catkin_package（DEPENDS）

* Contributors: Dirk Thomas, William Woodall

> *撰稿人：Dirk Thomas，William Woodall


1.7.6 (2012-10-30)

> 1.7.6（2012-10-30）
------------------

* fix catkin function order

> *固定catkin函数顺序

* Contributors: Dirk Thomas

> *投稿人：Dirk Thomas


1.7.5 (2012-10-27)

> 1.7.5（2012-10-27）
------------------

* clean up package.xml files

> *清理package.xml文件

* add missing target dependency to gencpp

> *将缺少的目标依赖项添加到gencpp

* Contributors: Dirk Thomas

> *投稿人：Dirk Thomas


1.7.4 (2012-10-06)

> 1.7.4（2012-10-06）
------------------


1.7.3 (2012-10-02 00:19)

> 1.7.3（2012-10-02 00:19）
------------------------

* fix package building issues

> *修复程序包构建问题

* Contributors: Vincent Rabaud

> *撰稿人：Vincent Rabaud


1.7.2 (2012-10-02 00:06)

> 1.7.2（2012-10-02 00:06）
------------------------

* add the missing catkin dependency

> *添加缺失的catkin依赖项

* Contributors: Vincent Rabaud

> *撰稿人：Vincent Rabaud


1.7.1 (2012-10-01 19:00)

> 1.7.1（2012-10-01 19:00）
------------------------

* add missing dependencies

> *添加缺少的依赖项

* Contributors: Vincent Rabaud

> *撰稿人：Vincent Rabaud


1.7.0 (2012-10-01 16:51)

> 1.7.0（2012-10-01 16:51）
------------------------

* catkinize bond

> *紧身债券

* catkinize the package and bump to 1.7.0 even though it is not tagged yet

> *catkinize the package and bump to 1.7.0即使它还没有被标记

* add link flag for OSX

> *为OSX添加链接标志

* removed spurious reference to libroslib

> *删除了对libroslib的虚假引用

* bondcpp now explicitly links against the ros library.  `#5334 <https://github.com/ros/bond_core/issues/5334>`_

> *bondcpp现在显式地链接到ros库`#5334个<https://github.com/ros/bond_core/issues/5334>`_

* Changed ros::Time/Duration to ros::WallTime/WallDuration so Bond still works when time stops.  Fixes `#5035 <https://github.com/ros/bond_core/issues/5035>`_

> *将ros：：Time/Duration更改为ros：：WallTime/WallDuration，因此当时间停止时，Bond仍然有效。修复`#5035<https://github.com/ros/bond_core/issues/5035>`_

* Fixed destruction bug: doesn't destroy things if the bond was never started.

> *修正了破坏错误：如果债券从未启动，就不会破坏东西。

* Can now set a bond's callback queue

> *现在可以设置债券的回调队列
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4037081

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/stack/common/ttrunk%4037081

* Modified bond's state machine to handle "alive" messages from the sibling when already dead.

> *修改了bond的状态机，以便在已经死亡的情况下处理来自兄弟姐妹的“活着”消息。
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036189

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/stack/common/ttrunk%403689

* Added global "bond_disable_heartbeat_timeout" parameter

> *添加全局“bond_disable_heartbeat_timeout”参数
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036106

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/stack/common/ttrunk%403606

* typo

> *打字错误
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035731

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/堆栈/公用/主干%4035731

* rosdep and packages are not the same

> *rosdep和软件包不一样
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035730

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/堆栈/公用/主干%4035730

* patch from stevenbellens for fedora uuid support `#4756 <https://github.com/ros/bond_core/issues/4756>`_

> *stevenbellens针对fedora uuid支持的补丁#4756<https://github.com/ros/bond_core/issues/4756>`_
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035729

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/堆栈/公用/主干%4035729

* Re-ordering locking in bondcpp's destructor

> *在bondcpp的析构函数中重新排序锁定
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035719

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/stack/common/ttrunk%4035719

* In bond, wait_until_formed and wait_until_broken terminate when ROS shuts down.

> *在债券中，当ROS关闭时，wait_util_formed和wait_util_broken终止。
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035632

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/ttrunk%4035632

* Bond no longer warns on destructor when the other side disappeared.

> *当另一边消失时，邦德不再警告毁灭者。
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035573

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/堆栈/公用/主干%4035573

* removed wiki syntax from description

> *从描述中删除了wiki语法
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035392

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/stack/common/ttrunk%4035392

* Creating package descriptions for bondpy, bondcpp, and test_bond.

> *为bondpy、bondcpp和test_bond创建包描述。
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035354

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/堆栈/公用/主干%4035534

* The bond state machine more gracefully handles excessive requests to die.

> *绑定状态机更优雅地处理过多的死亡请求。
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4032653

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/stack/common/ttrunk%4032653

* Moving bond into common

> *将债券转换为共同债券
  --HG--

  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4032634

> 额外：convert_revision:svn%3Web33c2ac-9c88-4c90-87e0-44a10359b0c3/stack/common/ttrunk%4032634

* Contributors: Brian Gerkey, Stuart Glaser, Vincent Rabaud, kwc, sglaser, tfoote

> *撰稿人：Brian Gerkey、Stuart Glaser、Vincent Rabaud、kwc、sglaser、tfoote
