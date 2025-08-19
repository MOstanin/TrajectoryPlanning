using System;
using Newtonsoft.Json;

namespace TrajectoryPlanning.Robot
{
    public class RobotModelDto
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

        [JsonConstructor]
        public RobotModelDto(
            string model,
            int dof,
            string[] jointNames,
            float[] jointMinRadians,
            float[] jointMaxRadians,
            float[] jointMaxVelocityRadPerSec,
            float[] jointMaxAccelerationRadPerSec2,
            float[] initialStateRadians
        )
        {
            this.model = model;
            this.dof = dof;
            this.jointNames = jointNames;
            this.jointMinRadians = jointMinRadians;
            this.jointMaxRadians = jointMaxRadians;
            this.jointMaxVelocityRadPerSec = jointMaxVelocityRadPerSec;
            this.jointMaxAccelerationRadPerSec2 = jointMaxAccelerationRadPerSec2;
            this.initialStateRadians = initialStateRadians;
        }
    }
}
