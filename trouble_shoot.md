
1. Cannot compile in mac Mojave

```
➜  build git:(master) ✗ cmake .. && make
-- Configuring done
-- Generating done
-- Build files have been written to: /Users/zhangxg/work/gitrepo/uda-sd/CarND-Unscented-Kalman-Filter-Project/build
Scanning dependencies of target UnscentedKF
[ 20%] Building CXX object CMakeFiles/UnscentedKF.dir/src/main.cpp.o
[ 40%] Linking CXX executable UnscentedKF
ld: warning: directory not found for option '-L/usr/local/Cellar/libuv/1*/lib'
Undefined symbols for architecture x86_64:
  "FileLoader::loadData()", referenced from:
      _main in test_boot.cpp.o
  "FileLoader::FileLoader(std::__1::basic_string<char, std::__1::char_traits<char>, std::__1::allocator<char> > const&)", referenced from:
      _main in test_boot.cpp.o
  "FileLoader::~FileLoader()", referenced from:
      _main in test_boot.cpp.o
ld: symbol(s) not found for architecture x86_64
clang: error: linker command failed with exit code 1 (use -v to see invocation)
make[2]: *** [UnscentedKF] Error 1
make[1]: *** [CMakeFiles/UnscentedKF.dir/all] Error 2
make: *** [all] Error 2

```

solution:
comment out the links in CMakeList.txt
