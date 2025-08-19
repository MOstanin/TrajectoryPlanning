using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json;
using TrajectoryPlanning.Utils;

namespace TrajectoryPlanning.Robot
{
    public class RobotModelDto : IWithId<string>
    {
        [JsonProperty("model")]
        public string model;

        [JsonProperty("dof")]
        public int dof;

        [JsonProperty("jointNames")]
        public string[] jointNames;

        [JsonProperty("jointMinRadians")]
        public float[] jointMinRadians;

        [JsonProperty("jointMaxRadians")]
        public float[] jointMaxRadians;

        [JsonProperty("jointMaxVelocityRadPerSec")]
        public float[] jointMaxVelocityRadPerSec;

        [JsonProperty("jointMaxAccelerationRadPerSec2")]
        public float[] jointMaxAccelerationRadPerSec2;

        [JsonProperty("initialStateRadians")]
        public float[] initialStateRadians;
        public string Id => model;

        [JsonConstructor]
        public RobotModelDto(
            string model,
            int dof,
            IEnumerable<string> jointNames,
            IEnumerable<float> jointMinRadians,
            IEnumerable<float> jointMaxRadians,
            IEnumerable<float> jointMaxVelocityRadPerSec,
            IEnumerable<float> jointMaxAccelerationRadPerSec2,
            IEnumerable<float> initialStateRadians
        )
        {
            this.model = model;
            this.dof = dof;
            this.jointNames = jointNames.ToArray();
            this.jointMinRadians = jointMinRadians.ToArray();
            this.jointMaxRadians = jointMaxRadians.ToArray();
            this.jointMaxVelocityRadPerSec = jointMaxVelocityRadPerSec.ToArray();
            this.jointMaxAccelerationRadPerSec2 = jointMaxAccelerationRadPerSec2.ToArray();
            this.initialStateRadians = initialStateRadians.ToArray();
        }
    }
}
