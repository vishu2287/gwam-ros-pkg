# Instructions for Updating the Guardian-WAM Repository to the Latest Release #

When working with the Guardian-WAM System you may need functionality that may not have been implemented in earlier versions of the software.

The following is instructions for updating the gwam-ros-pkg on a user PC, the Guardian PC, or the WAM PC.

gwam-ros-pkg uses Apache Subversion (svn) for its software versioning and revision control system.  More information and tutorials regarding subversion can be found [here](http://subversion.apache.org/).

### Instructions ###

Updating gwam-ros-pkg is as simple as using the standard svn up, and recompiling.

- SSH into the computer or robot you would like to update.

- Navigate to the gwam-ros-pkg and update
```
roscd gwam-ros-pkg
svn up
```

- Recompile the appropriate portions of your software using rosmake