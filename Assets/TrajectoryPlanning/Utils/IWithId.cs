namespace TrajectoryPlanning.Utils
{
    public interface IWithId<out TId>
    {
        TId Id { get; }
    }
}
