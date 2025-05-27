using System.Collections.Generic;
using Taichi.Generated;

namespace Taichi {

    public class ArgumentListBuilder {
        private List<TiArgument> _Args;

        public ArgumentListBuilder() {
            _Args = new List<TiArgument>();
        }

        public ArgumentListBuilder Add(int x) {
            var arg = new TiArgument {
                type = TiArgumentType.TI_ARGUMENT_TYPE_I32,
                value = new TiArgumentValue {
                    i32 = x,
                },
            };
            _Args.Add(arg);
            return this;
        }
        public ArgumentListBuilder Add(float x) {
            var arg = new TiArgument {
                type = TiArgumentType.TI_ARGUMENT_TYPE_F32,
                value = new TiArgumentValue {
                    f32 = x,
                },
            };
            _Args.Add(arg);
            return this;
        }
        public ArgumentListBuilder Add<T>(NdArray<T> x) where T : struct {
            var arg = new TiArgument {
                type = TiArgumentType.TI_ARGUMENT_TYPE_NDARRAY,
                value = new TiArgumentValue {
                    ndarray = x.ToTiNdArray(),
                },
            };
            _Args.Add(arg);
            return this;
        }

        public TiArgument[] ToArray() {
            return _Args.ToArray();
        }
    }

    public class NamedArgumentListBuilder {
        private List<TiNamedArgument> _NamedArgs;

        public NamedArgumentListBuilder() {
            _NamedArgs = new List<TiNamedArgument>();
        }

        public NamedArgumentListBuilder Add(string name, int x) {
            var arg = new TiArgument {
                type = TiArgumentType.TI_ARGUMENT_TYPE_I32,
                value = new TiArgumentValue {
                    i32 = x,
                },
            };
            _NamedArgs.Add(new TiNamedArgument {
                name = name,
                argument = arg,
            });
            return this;
        }
        public NamedArgumentListBuilder Add(string name, float x) {
            var arg = new TiArgument {
                type = TiArgumentType.TI_ARGUMENT_TYPE_F32,
                value = new TiArgumentValue {
                    f32 = x,
                },
            };
            _NamedArgs.Add(new TiNamedArgument {
                name = name,
                argument = arg,
            });
            return this;
        }
        public NamedArgumentListBuilder Add<T>(string name, NdArray<T> x) where T : struct {
            var arg = new TiArgument {
                type = TiArgumentType.TI_ARGUMENT_TYPE_NDARRAY,
                value = new TiArgumentValue {
                    ndarray = x.ToTiNdArray(),
                },
            };
            _NamedArgs.Add(new TiNamedArgument {
                name = name,
                argument = arg,
            });
            return this;
        }

        public TiNamedArgument[] ToArray() {
            return _NamedArgs.ToArray();
        }
    }

}
