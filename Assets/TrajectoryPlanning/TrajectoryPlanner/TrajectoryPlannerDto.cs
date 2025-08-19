using Newtonsoft.Json;
using TrajectoryPlanning.Utils;

namespace TrajectoryPlanning.Planner
{
	public class TrajectoryPlannerDto : IWithId<string>
	{
		[JsonProperty("id")]
		public string id;

		[JsonProperty("rate")]
		public float rate;

		public string Id => id;

		[JsonConstructor]
		public TrajectoryPlannerDto(string id, float rate)
		{
			this.id = id;
			this.rate = rate;
		}
	}
}


