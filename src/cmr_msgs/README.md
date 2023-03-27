# cmr_msgs

@brief This package contains all the messages (`*.msg`), services (`.srv`), and
actions (`*.action`) that are defined and used by CMR packages.

The reason we put them all in the same package, rather than having each package
contain its own such files, is because this would require declaring a dependency
on those individual packages whenever a package wants to use one of these files.
This can get messy because it might lead us straight into a dependency mess with
unnecessary and cyclical dependencies. This way, we can just declare a dependency
on "cmr_msgs" and avoid the fuss.
