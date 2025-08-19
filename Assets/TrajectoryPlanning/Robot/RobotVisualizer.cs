using System;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using UniRx;
using UnityEngine;
using Zenject;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

namespace TrajectoryPlanning.Robot
{
    public class RobotVisualizer : MonoBehaviour
    {
        public enum Axis
        {
            None,
            X,
            Y,
            Z
        }

        [Serializable]
        public class RevoluteJoint
        {
            [Tooltip("The Transform that represents this revolute joint.")]
            public Transform transform;

            [Tooltip("Local axis around which this joint rotates.")]
            public Axis axis = Axis.Z;
        }

        [Header("Robot Joints (6 DOF)")]
        [SerializeField]
        private RevoluteJoint[] joints = new RevoluteJoint[6];

        [Header("End Effector (optional)")]
        [SerializeField]
        private Transform endEffector;

        [Inject]
        public void Construct(IReadOnlyRobotModel robotModel)
        {
            robotModel.CurrentState.Subscribe(SetState).AddTo(this);
        }

        private void SetState(Vector<float> state)
        {
            if (joints == null || joints.Length == 0)
                throw new InvalidOperationException("No joints assigned");

            if (joints.Length != state.Count)
                throw new InvalidOperationException("Invalid state vector length");

            for (var i = 0; i < joints.Length; i++)
            {
                var joint = joints[i];
                if (joint == null || joint.transform == null || joint.axis == Axis.None)
                    throw new InvalidOperationException("Invalid joint configuration");

                var angleRad = state[i];
                var angleDeg = angleRad * Mathf.Rad2Deg;

                var axisVector = joint.axis switch
                {
                    Axis.X => Vector3.right,
                    Axis.Y => Vector3.up,
                    Axis.Z => Vector3.forward,
                    _ => throw new ArgumentOutOfRangeException()
                };

                joint.transform.localRotation = Quaternion.AngleAxis(angleDeg, axisVector);
            }
        }
    }
}
