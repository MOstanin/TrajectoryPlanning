using Zenject;

namespace TrajectoryPlanning.Robot
{
    public class RobotFactory : IFactory<string, RobotModel>
    {
        public RobotModel Create(string id)
        {
            throw new System.NotImplementedException();
        }
    }
}
