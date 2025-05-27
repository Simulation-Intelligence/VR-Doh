using System.Collections.Generic;

namespace Taichi {
    public static class AotModuleRegistry {
        private static Dictionary<string, AotModule> _Registry;

        static AotModuleRegistry() {
            _Registry = new Dictionary<string, AotModule>();
        }


        public static void Register(string guid, AotModule module) {
            if (!_Registry.ContainsKey(guid)) {
                _Registry.Add(guid, module);
            }
        }
        public static void Unregister(string guid) {
            if (_Registry.ContainsKey(guid)) {
                _Registry[guid].Dispose();
                _Registry.Remove(guid);
            }
        }

        public static AotModule Get(string guid) {
            return _Registry[guid];
        }
    }
}
