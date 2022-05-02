#!/usr/bin/env python
# -*- coding: utf-8 -*-

import intra_test.mycpplib2 as cpp

def pycount(obj): import sys; return sys.getrefcount(obj) - 3

if __name__=="__main__":
    print("Before Construction")
    cpp.SharedResource.get_count()
    objs = [cpp.SharedResource.make() for it in range(10)]
    print("After Construction")
    cpp.SharedResource.get_count()

    print("  pycount(obj) =", pycount(objs[1]))

    del objs
    cpp.SharedResource.get_count()