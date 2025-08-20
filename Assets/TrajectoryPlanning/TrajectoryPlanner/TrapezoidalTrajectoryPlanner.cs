using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanning.Robot;
using UniRx;
using UnityEngine;

namespace TrajectoryPlanning.TrajectoryPlanner
{
    public class TrapezoidalTrajectoryPlanner : ITrajectoryPlanner, IDisposable
    {
        private const float Eps = 1e-6f;
        private readonly ReactiveProperty<float> _rate;
        public string Id { get; }
        public IReactiveProperty<float> Rate => _rate;

        public TrapezoidalTrajectoryPlanner(string id, float rate)
        {
            _rate = new ReactiveProperty<float>(rate);
            Id = id;
        }

        public Trajectory PlanMoveJ(
            IReadOnlyRobotModel model,
            Vector<float> q0,
            Vector<float> qTarget
        )
        {
            var degreesOfFreedom = model.Dof;
            var jointVelocityLimits = model.JointsMaxVelocity.Value;
            var jointAccelerationLimits = model.JointsMaxAcceleration.Value;

            if (q0.Count != degreesOfFreedom || qTarget.Count != degreesOfFreedom)
                throw new ArgumentException("State size does not match robot DOF");

            var timeStep = 1 / Rate.Value;

            ComputeBaseProfiles(
                degreesOfFreedom,
                q0,
                qTarget,
                jointVelocityLimits,
                jointAccelerationLimits,
                out var dqAbs,
                out var sign,
                out var tAcc,
                out var tCruise,
                out var tDec,
                out var vPeak,
                out var total
            );

            var totalDuration = total.Max();
            var timeSamples = BuildTimeSamples(totalDuration, timeStep);
            Debug.Log(
                $"Trajectory took {timeSamples.Length} time samples, dt={timeStep:F2}s, T={totalDuration:F2}s"
            );

            ComputePhasesForSharedTime(
                degreesOfFreedom,
                jointVelocityLimits,
                jointAccelerationLimits,
                dqAbs,
                tAcc,
                tCruise,
                tDec,
                vPeak,
                total,
                totalDuration,
                out var tAccScaled,
                out var tCruiseScaled,
                out var tDecScaled,
                out var vPeakScaled,
                out var accScaled
            );

            var trajectory = SampleTrajectory(
                degreesOfFreedom,
                timeSamples,
                q0,
                qTarget,
                dqAbs,
                sign,
                tAccScaled,
                tCruiseScaled,
                tDecScaled,
                vPeakScaled,
                accScaled
            );

            Debug.Log(
                $"Trajectory generated for {model.Id} totally {timeSamples.Length} steps, T={totalDuration:F2}s"
            );
            return trajectory;
        }

        private static float[] BuildTimeSamples(float T, float dt)
        {
            var times = new List<float>();
            var t = 0f;
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

        private static void ComputeBaseProfiles(
            int degreesOfFreedom,
            Vector<float> q0,
            Vector<float> qTarget,
            float[] jointVelocityLimits,
            float[] jointAccelerationLimits,
            out float[] absoluteDelta,
            out float[] directionSign,
            out float[] accelTime,
            out float[] cruiseTime,
            out float[] decelTime,
            out float[] peakVelocity,
            out float[] jointTotalTime
        )
        {
            absoluteDelta = new float[degreesOfFreedom];
            directionSign = new float[degreesOfFreedom];
            accelTime = new float[degreesOfFreedom];
            cruiseTime = new float[degreesOfFreedom];
            decelTime = new float[degreesOfFreedom];
            peakVelocity = new float[degreesOfFreedom];
            jointTotalTime = new float[degreesOfFreedom];

            for (var i = 0; i < degreesOfFreedom; i++)
            {
                var vLim = Math.Max(jointVelocityLimits[i], Eps);
                var aLim = Math.Max(jointAccelerationLimits[i], Eps);
                var dq = qTarget[i] - q0[i];
                absoluteDelta[i] = Math.Abs(dq);
                directionSign[i] = Math.Sign(absoluteDelta[i]) == 0 ? 0f : Math.Sign(dq);

                if (absoluteDelta[i] <= Eps)
                {
                    accelTime[i] = 0f;
                    cruiseTime[i] = 0f;
                    decelTime[i] = 0f;
                    peakVelocity[i] = 0f;
                    jointTotalTime[i] = 0f;
                    continue;
                }

                var tToVmax = vLim / aLim;
                var dAccel = 0.5f * aLim * (float)Math.Pow(tToVmax, 2);

                if (absoluteDelta[i] < 2f * dAccel)
                {
                    var tAccTri = (float)Math.Sqrt(absoluteDelta[i] / aLim);
                    accelTime[i] = tAccTri;
                    cruiseTime[i] = 0f;
                    decelTime[i] = tAccTri;
                    peakVelocity[i] = aLim * tAccTri;
                    jointTotalTime[i] = 2f * tAccTri;
                }
                else
                {
                    accelTime[i] = tToVmax;
                    var dCruise = absoluteDelta[i] - 2f * dAccel;
                    cruiseTime[i] = dCruise / vLim;
                    decelTime[i] = tToVmax;
                    peakVelocity[i] = vLim;
                    jointTotalTime[i] = 2f * tToVmax + cruiseTime[i];
                }
            }
        }

        private static void ComputePhasesForSharedTime(
            int degreesOfFreedom,
            float[] jointVelocityLimits,
            float[] jointAccelerationLimits,
            float[] absoluteDelta,
            float[] accelTimeBase,
            float[] cruiseTimeBase,
            float[] decelTimeBase,
            float[] peakVelocityBase,
            float[] jointTotalTime,
            float totalDuration,
            out float[] accelTime,
            out float[] cruiseTime,
            out float[] decelTime,
            out float[] peakVelocity,
            out float[] acceleration
        )
        {
            accelTime = new float[degreesOfFreedom];
            cruiseTime = new float[degreesOfFreedom];
            decelTime = new float[degreesOfFreedom];
            peakVelocity = new float[degreesOfFreedom];
            acceleration = new float[degreesOfFreedom];

            for (var i = 0; i < degreesOfFreedom; i++)
            {
                if (absoluteDelta[i] <= Eps)
                {
                    acceleration[i] = 0f;
                    peakVelocity[i] = 0f;
                    accelTime[i] = 0f;
                    cruiseTime[i] = totalDuration;
                    decelTime[i] = 0f;
                    continue;
                }

                var vLim = Math.Max(jointVelocityLimits[i], Eps);
                var aLim = Math.Max(jointAccelerationLimits[i], Eps);
                var d = absoluteDelta[i];

                var vReq = 2f * d / Math.Max(totalDuration, Eps);
                var aReq = 4f * d / Math.Max(totalDuration * totalDuration, Eps);

                if (vReq <= vLim + Eps && aReq <= aLim + Eps)
                {
                    accelTime[i] = totalDuration * 0.5f;
                    cruiseTime[i] = 0f;
                    decelTime[i] = totalDuration * 0.5f;
                    peakVelocity[i] = vReq;
                    acceleration[i] = aReq;
                }
                else
                {
                    var denom = vLim * totalDuration - d;
                    if (denom > Eps)
                    {
                        var aCalc = (vLim * vLim) / denom;
                        if (aCalc > Eps && aCalc <= aLim + Eps)
                        {
                            var t1 = vLim / aCalc;
                            var tc = Math.Max(0f, totalDuration - 2f * t1);
                            accelTime[i] = t1;
                            cruiseTime[i] = tc;
                            decelTime[i] = t1;
                            peakVelocity[i] = vLim;
                            acceleration[i] = aCalc;
                            continue;
                        }
                    }

                    var k = jointTotalTime[i] <= Eps ? 1f : totalDuration / jointTotalTime[i];
                    accelTime[i] = accelTimeBase[i] * k;
                    cruiseTime[i] = cruiseTimeBase[i] * k;
                    decelTime[i] = decelTimeBase[i] * k;
                    peakVelocity[i] = peakVelocityBase[i] / k;
                    acceleration[i] = accelTime[i] <= Eps ? 0f : peakVelocity[i] / accelTime[i];
                }
            }
        }

        private static Trajectory SampleTrajectory(
            int degreesOfFreedom,
            float[] timeSamples,
            Vector<float> q0,
            Vector<float> qTarget,
            float[] absoluteDelta,
            float[] directionSign,
            float[] accelTime,
            float[] cruiseTime,
            float[] decelTime,
            float[] peakVelocity,
            float[] acceleration
        )
        {
            var positions = new Vector<float>[timeSamples.Length];
            var velocities = new Vector<float>[timeSamples.Length];
            var accelerations = new Vector<float>[timeSamples.Length];

            for (var kIdx = 0; kIdx < timeSamples.Length; kIdx++)
            {
                var tNow = timeSamples[kIdx];
                var qArr = new float[degreesOfFreedom];
                var vArr = new float[degreesOfFreedom];
                var aArr = new float[degreesOfFreedom];

                for (var i = 0; i < degreesOfFreedom; i++)
                {
                    if (absoluteDelta[i] <= Eps)
                    {
                        qArr[i] = q0[i];
                        vArr[i] = 0f;
                        aArr[i] = 0f;
                        continue;
                    }

                    var sgn = directionSign[i];
                    var a = acceleration[i];
                    var vp = peakVelocity[i];
                    var t1 = accelTime[i];
                    var tc = cruiseTime[i];
                    var t2 = decelTime[i];

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

            return new Trajectory(positions, velocities, accelerations, timeSamples);
        }

        public void Dispose()
        {
            _rate.Dispose();
        }
    }
}
