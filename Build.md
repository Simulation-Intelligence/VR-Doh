# How to build Taichi Unity examples
Because Taichi AOT is in active development, the Unity examples might suffer from unintentional breakage. This documentation helps you to recover from the incidents if we cannot immediately address your issue in the first place.

## Assertion failed in `serialization.h`

It means that `taichi_c_api.dll` and `taichi_unity.dll` in `Assets/Plugins/X86_64` are incompatible with the AOT modules in `Assets/Resources/TaichiModules`. You should recompile the AOT modules with programs in `scripts`, and update the DLLs from a same version of Taichi.

For implicit_fem demo, you have to make sure that the `--aot` parameter in the corresponding `.py` file is set to **True**. This ensures that AOT module is compiled correctly. 

You can find `taichi_c_api.dll` in [a nightly Python wheel](https://pypi.org/project/taichi-nightly/). Or you can build it from source following the steps in [Developer Installation](https://docs.taichi-lang.org/docs/dev_install) with CMake definition `TI_WITH_C_API=ON`. The Taichi Runtime C-API library can be found in `taichi/_skbuild/<platform>/cmake-install/c_api`.

To ensure you are building with `TI_WITH_C_API`, please lookup the CMake options in `taichi/_skbuild/<platform>/cmake-build/CMakeCache.txt`.After recompiling, you can find the dll file in the c_api folder mentioned earlier.And you can get `taichi_unity.dll` from [taichi-dev/taichi-unity2](https://github.com/taichi-dev/taichi-unity2) in a similar way.

## InvalidOperationException: Ignored launch because kernel handle is null

It's rooted from the same compatibility issue as the one above. You should recompile the AOT modules with programs in scripts, and update the DLLs from a same version of Taichi.

