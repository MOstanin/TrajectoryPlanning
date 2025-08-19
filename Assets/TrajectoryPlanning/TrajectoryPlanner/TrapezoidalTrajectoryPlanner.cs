using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanning.Robot;
using TrajectoryPlanning.TrajectoryPlanner;

namespace TrajectoryPlanning.Planner
{
    public sealed class TrapezoidalTrajectoryPlanner : ITrajectoryPlanner
    {
        public string Id { get; }
        public float Rate { get; }

        public TrapezoidalTrajectoryPlanner(string id, float rate)
        {
            Rate = rate;
            Id = id;
        }

        public Trajectory PlanMoveJ(
            IReadOnlyRobotModel model,
            Vector<float> q0,
            Vector<float> qTarget
        )
        {
            var dof = model.Dof;
            var maxVel = model.JointsMaxVelocity.Value;
            var maxAcc = model.JointsMaxAcceleration.Value;

            if (q0.Count != dof || qTarget.Count != dof)
                throw new ArgumentException("State size does not match robot DOF");

            const float eps = 1e-6f;
            var dt = 1 / Rate;

            var dqAbs = new float[dof];
            var sign = new float[dof];
            var tAcc = new float[dof];
            var tCruise = new float[dof];
            var tDec = new float[dof];
            var vPeak = new float[dof];
            var total = new float[dof];

            for (var i = 0; i < dof; i++)
            {
                var vLim = Math.Max(maxVel[i], eps);
                var aLim = Math.Max(maxAcc[i], eps);
                var dq = qTarget[i] - q0[i];
                dqAbs[i] = Math.Abs(dq);
                sign[i] = Math.Sign(dqAbs[i]) == 0 ? 0f : Math.Sign(dq);

                if (dqAbs[i] <= eps)
                {
                    tAcc[i] = 0f;
                    tCruise[i] = 0f;
                    tDec[i] = 0f;
                    vPeak[i] = 0f;
                    total[i] = 0f;
                    continue;
                }

                var tToVmax = vLim / aLim;
                var dAccel = 0.5f * aLim * (float)Math.Pow(tToVmax, 2);

                if (dqAbs[i] < 2f * dAccel)
                {
                    var tAccTri = (float)Math.Sqrt(dqAbs[i] / aLim);
                    tAcc[i] = tAccTri;
                    tCruise[i] = 0f;
                    tDec[i] = tAccTri;
                    vPeak[i] = aLim * tAccTri;
                    total[i] = 2f * tAccTri;
                }
                else
                {
                    tAcc[i] = tToVmax;
                    var dCruise = dqAbs[i] - 2f * dAccel;
                    tCruise[i] = dCruise / vLim;
                    tDec[i] = tToVmax;
                    vPeak[i] = vLim;
                    total[i] = 2f * tToVmax + tCruise[i];
                }
            }

            var T = total.Max();
            var time = BuildTimeSamples(T, dt);

            var accScaled = new float[dof];
            var vPeakScaled = new float[dof];
            var tAccScaled = new float[dof];
            var tCruiseScaled = new float[dof];
            var tDecScaled = new float[dof];

            for (var i = 0; i < dof; i++)
            {
                if (dqAbs[i] <= eps)
                {
                    accScaled[i] = 0f;
                    vPeakScaled[i] = 0f;
                    tAccScaled[i] = 0f;
                    tCruiseScaled[i] = T;
                    tDecScaled[i] = 0f;
                    continue;
                }

                var k = total[i] <= eps ? 1f : T / total[i];
                tAccScaled[i] = tAcc[i] * k;
                tCruiseScaled[i] = tCruise[i] * k;
                tDecScaled[i] = tDec[i] * k;
                vPeakScaled[i] = vPeak[i] / k;
                accScaled[i] = tAccScaled[i] <= eps ? 0f : vPeakScaled[i] / tAccScaled[i];
            }

            var positions = new Vector<float>[time.Length];
            var velocities = new Vector<float>[time.Length];
            var accelerations = new Vector<float>[time.Length];

            for (var kIdx = 0; kIdx < time.Length; kIdx++)
            {
                var tNow = time[kIdx];
                var qArr = new float[dof];
                var vArr = new float[dof];
                var aArr = new float[dof];

                for (var i = 0; i < dof; i++)
                {
                    if (dqAbs[i] <= eps)
                    {
                        qArr[i] = q0[i];
                        vArr[i] = 0f;
                        aArr[i] = 0f;
                        continue;
                    }

                    var sgn = sign[i];
                    var a = accScaled[i];
                    var vp = vPeakScaled[i];
                    var t1 = tAccScaled[i];
                    var tc = tCruiseScaled[i];
                    var t2 = tDecScaled[i];

                    if (tNow <= t1)
                    {
                        var tt = tNow;
                        qArr[i] = q0[i] + sgn * (0.5f * a * tt * tt);
                        vArr[i] = sgn * (a * tt);
                        aArr[i] = sgn * a;
                    }
                    else if (tNow <= t1 + tc)
                    {
                        var tt = tNow - t1;
                        qArr[i] = q0[i] + sgn * (0.5f * a * t1 * t1 + vp * tt);
                        vArr[i] = sgn * vp;
                        aArr[i] = 0f;
                    }
                    else
                    {
                        var tt = Math.Min(tNow - (t1 + tc), t2);
                        var qCruiseEnd = q0[i] + sgn * (0.5f * a * t1 * t1 + vp * tc);
                        qArr[i] = qCruiseEnd + sgn * (vp * tt - 0.5f * a * tt * tt);
                        if (tNow >= t1 + tc + t2)
                        {
                            qArr[i] = qTarget[i];
                            vArr[i] = 0f;
                            aArr[i] = 0f;
                        }
                        else
                        {
                            vArr[i] = sgn * (vp - a * tt);
                            aArr[i] = -sgn * a;
                        }
                    }
                }

                positions[kIdx] = Vector<float>.Build.Dense(qArr);
                velocities[kIdx] = Vector<float>.Build.Dense(vArr);
                accelerations[kIdx] = Vector<float>.Build.Dense(aArr);
            }

            return new Trajectory(positions, velocities, accelerations, time);
        }

        private static float[] BuildTimeSamples(float T, float dt)
        {
            var times = new List<float>();
            var t = 0f;
            if (dt <= 0f)
                dt = 0.01f;
            var nMax = (int)Math.Ceiling(T / dt) + 2;
            for (var i = 0; i < nMax; i++)
            {
                times.Add(t);
                t += dt;
                if (t >= T)
                    break;
            }
            if (times.Count == 0 || Math.Abs(times[^1] - T) > 1e-5f)
                times.Add(T);
            return times.ToArray();
        }
    }
}
