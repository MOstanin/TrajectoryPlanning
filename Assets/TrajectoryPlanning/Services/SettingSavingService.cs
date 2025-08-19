using System.IO;
using Newtonsoft.Json;
using OneOf;
using OneOf.Types;
using UnityEngine;

namespace TrajectoryPlanning.Services
{
    public interface ISettingSavingService<TDto>
        where TDto : class
    {
        OneOf<TDto, None> Load(string id);
        void Save(TDto dto);
    }

    public sealed class SettingSavingService<TDto> : ISettingSavingService<TDto>
        where TDto : class
    {
        private readonly JsonSerializer _serializer;
        private readonly string _relativePath;

        public SettingSavingService(JsonSerializer serializer, string relativePath)
        {
            _serializer = serializer;
            _relativePath = relativePath;
        }

        public OneOf<TDto, None> Load(string id)
        {
            var persistPath = Path.Combine(
                Application.persistentDataPath,
                _relativePath,
                id + ".json"
            );
            if (File.Exists(persistPath))
            {
                using var sr = new StreamReader(persistPath);
                using var reader = new JsonTextReader(sr);
                var deserializedData = _serializer.Deserialize<TDto>(reader);
                return deserializedData != null
                    ? OneOf<TDto, None>.FromT0(deserializedData)
                    : new None();
            }

            var textAsset = Resources.Load<TextAsset>(_relativePath);
            if (textAsset != null)
            {
                using var sr = new StringReader(textAsset.text);
                using var reader = new JsonTextReader(sr);
                var deserializedData = _serializer.Deserialize<TDto>(reader);
                return deserializedData != null
                    ? OneOf<TDto, None>.FromT0(deserializedData)
                    : new None();
            }

            return new None();
        }

        public void Save(TDto dto)
        {
            var persistPath = Path.Combine(Application.persistentDataPath, _relativePath);
            var directory = Path.GetDirectoryName(persistPath);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
                Directory.CreateDirectory(directory);

            using var sw = new StreamWriter(persistPath);
            using var writer = new JsonTextWriter(sw);
            writer.Formatting = Formatting.Indented;
            _serializer.Serialize(writer, dto);
        }
    }
}
