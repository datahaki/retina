# Guidelines for Software Development

*Target audience*: Software developers who are contributing to the gokart project, i.e. a safety critical application

## Modality

* Make your work visible: `push` your code modifications to `github` no later than at the end of the day. That way, reviewers can give feedback immediately and coordinate the work better.
* Only commit your own source code.
* If you adapt a code snippet from another source, state the origin and give credit, for instance via a URL.
* Do not commit a library that was created by a 3rd party. Instead, such a repository can be forked and maintained separately.
* Do not commit *binary files*, for instance: compiler output, log files, image files, `PDF`s, ... The exception are files (less than `100[kB]`) that are for testing purposes and that are unlikely to change in the future.
* Please do not *rename* or *move* files authored by another developer. Instead, pass on your suggestion for refactoring to the software architect.

## Scope

The staff supports you to make your project a success.
Your Thesis-/Semester project builds on the work of others.
We encourage and expect you to make some contributions outside the scope of your project.
In the `github` issues, there is a list of desirable features that the whole group would benefit from.
Besides these ideas there are actions that advance the gokart project without the risk of breaking existing functionality: inserting comments and writing tests.
In conclusion: please spend some time on code items that may not be the highlights of your report, but are appreciated by the group nevertheless.

## Design

* many small source files (as opposed to large files)
* functions with few lines of code
* modularity
* minimal redundancy
* use of immutable objects
* tests and test coverage
* uniform code format (automatic)

Typically, these objectives are not achieved in the first version of the implementation.
The staff will give you suggestions on how to modify your code in order to come closer to the above standards.
Please be give the suggestions of the reviewers a high priority in your schedule.
We operate the gokart using code that adheres to best practices.

## Language

The software framework of the gokart project allows the use of different languages.
The processes exchange packets of information via the message passing standard `LCM`.
A single type of message is used: a byte array of variable length in little-endian encoding.

### Java 8

So far, the gokart code is written in `Java 8`

* sensor and actuator interfaces
* emergency modules
* controllers for steering, velocity, and trajectory following
* localization by lidar
* motion planning
* visualization in 2D and 3D

### Python

The use of `Python` *may* be advantageous for modules that involve

* camera calibration
* image processing
* machine learning

### C++

There are at least 6 ways to initialize a pointer to be *null* in `c++`

    myclass() : ptr {} { ...
    myclass() : ptr {0} { ...
    myclass() : ptr {nullptr} { ...
    myclass() : ptr ({}) { ...
    myclass() : ptr ({0}) { ...
    myclass() : ptr (nullptr) { ...