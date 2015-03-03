Support Vector Data Description(SVDD) package
Hyung Jin Chang 
hjchang@neuro.snu.ac.kr


Description
===========

This MATLAB package implements the mehods for exact SVDD batch learning, incremental SVDD learning, SVDD result drawing and Fast Increment learning for SVDD using Sample Margin in "Fast Incremental Learning for One-class Support Vector Classifier using Sample Margin Information" by Pyo Jae Kim, Hyung Jin Chang and Jin Young Chang. The PDF file of the paper is included with the code distribution.  

This code is designed for training SVDDs to solve one class classification problems. 

The primary benefits of this code are:
* exact incremental learning - one or more examples can be exactly incremented into the current SVDD solution, resulting in a classifier that is valid for the entire training set seen up to that point


Installation
============

Simply place the collection of m-files in a directory that is in MATLAB's path.  To verify everything is working properly, execute the commands listed in the file testresults.txt.  You should be able to replicate the output shown in this file.



Routines
========

The following m-files are the main functions supporting SVDD learning. 

SVDDtrain_batch - This m-file trains SVDD in batch type.
incSVDD - incSVDD performs incremental learning.

SVDDdrawing - Plotting the trained result in 2 dimension or 3 dimension

CalSampleMargin - Calculating data's sample margin

IncSVDD_demo - Demonstration code for usage of IncSVDD.m
FastIncSVDD_demo - Demonstration code of Fast Inc SVDD algorithm



Questions/Comments
==================

Drop me a line at the email address above!



License
=======

Copyright (C) 2009  Hyung Jin Chang

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.