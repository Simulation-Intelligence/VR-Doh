using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using UnityEngine;

namespace Taichi
{
    public class AotModuleAsset : ScriptableObject
    {
        public string Guid;
        public byte[] ArchiveData;

        private string _UnzipDir => $"{Application.temporaryCachePath}/tcm_{Guid}";
        private string _ContentListPath => $"{_UnzipDir}/__content__";



        public void Load()
        {
            var dstDir = _UnzipDir;
            if (!File.Exists(_ContentListPath))
            {
                using (var stream = new MemoryStream(ArchiveData))
                using (var archive = new ZipArchive(stream))
                {
                    archive.ExtractToDirectory(dstDir);
                }
            }
            AotModuleRegistry.Register(Guid, new AotModule(dstDir));
        }
        public void Unload()
        {
            AotModuleRegistry.Unregister(Guid);
        }


        public AotModule Inner
        {
            get
            {
                Load();
                return AotModuleRegistry.Get(Guid);
            }
        }


        public Kernel GetKernel(string name)
        {
            return Inner.GetKernel(name);
        }
        public ComputeGraph GetComputeGraph(string name)
        {
            return Inner.GetComputeGraph(name);
        }

        public Kernel[] GetAllKernels()
        {
            Load();
            List<string> names = new List<string>();
            using (var stream = File.OpenRead(_ContentListPath))
            using (var sr = new StreamReader(stream))
            {
                while (!sr.EndOfStream)
                {
                    var line = sr.ReadLine();
                    if (line.StartsWith("kernel:"))
                    {
                        names.Add(line.Substring(7));
                    }
                }
            }
            Kernel[] kernels = new Kernel[names.Count];
            for (int i = 0; i < kernels.Length; ++i)
            {
                kernels[i] = GetKernel(names[i]);
            }
            return kernels;
        }
        public ComputeGraph[] GetAllComputeGrpahs()
        {
            Load();
            List<string> names = new List<string>();
            using (var stream = File.OpenRead(_ContentListPath))
            using (var sr = new StreamReader(stream))
            {
                while (!sr.EndOfStream)
                {
                    var line = sr.ReadLine();
                    if (line.StartsWith("cgraph:"))
                    {
                        names.Add(line.Substring(7));
                    }
                }
            }
            ComputeGraph[] cgraphs = new ComputeGraph[names.Count];
            for (int i = 0; i < cgraphs.Length; ++i)
            {
                cgraphs[i] = GetComputeGraph(names[i]);
            }
            return cgraphs;
        }
    }
}
