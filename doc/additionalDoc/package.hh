/** \mainpage
\section sec_intro Introduction

This library implements a modular architecture to test and experiment
controllers in the Stack of Tasks Framework as defined in \ref Mansard2007.
It is specifically targeted to retain real-time capabilities while
having high level software capabilities such as:
\li On-line plugin loading and unloading,
\li Scripting of the control,
\li A strong software model which allow a clear identification
of the library application field.

\section sec_RequirementsInstallation Requirements and Installation
 The library assumes that Boost is installed.

\section sec_organization Organization of the code

The code is divided mostly in two parts: the controller also called
Tasks, and the signals providing the information necessary to compute
the command.
