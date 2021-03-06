using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic rigid-body model component which can be dropped onto
/// a game object.
/// </summary>
public class RigidBody : MonoBehaviour, ISimulable
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public RigidBody()
    {
        Manager = null;
    }

    #region EditorVariables

    public float Mass;
    public float Damping;

    #endregion

    #region OtherVariables

    private PhysicsManager Manager;

    public int index;

    public Vector3 m_pos;
    public Quaternion m_rot;
    public Vector3 m_vel;
    public Vector3 m_omega;

    protected MatrixXD m_inertia0, m_inertia0inv;
    protected MatrixXD m_inertia, m_inertiainv;

    #endregion

    #region MonoBehaviour

    // Update is called once per frame
    void Update()
    {
        // Apply the position and rotation to the mesh
        Transform xform = GetComponent<Transform>();
        xform.position = m_pos;
        xform.rotation = m_rot;
    }

    #endregion

    #region ISimulable

    public void Initialize(int ind, PhysicsManager m)
    {
        Manager = m;

        index = ind;

        // Initialize inertia. We assume that the object is connected to a Cube mesh.
        Transform xform = GetComponent<Transform>();
        if (xform == null)
        {
            System.Console.WriteLine("[ERROR] Couldn't find any transform to the rigid body");
        }
        else
        {
            System.Console.WriteLine("[TRACE] Succesfully found transform connected to the rigid body");
        }

        if (xform != null)
        {
            m_inertia0 = DenseMatrixXD.CreateIdentity(3);
            double[] vals;
            vals = new double[3];
            vals[0] = 1.0f / 12.0f * Mass * (xform.localScale.y * xform.localScale.y + xform.localScale.z * xform.localScale.z);
            vals[1] = 1.0f / 12.0f * Mass * (xform.localScale.x * xform.localScale.x + xform.localScale.z * xform.localScale.z);
            vals[2] = 1.0f / 12.0f * Mass * (xform.localScale.x * xform.localScale.x + xform.localScale.y * xform.localScale.y);
            m_inertia0.SetDiagonal(vals);
        }

        // Initialize kinematics
        m_pos = xform.position;
        m_rot = xform.rotation;
        m_vel = Vector3.zero;
        m_omega = Vector3.zero;

        // Initialize all inertia terms
        m_inertia0inv = m_inertia0.Inverse();
        m_inertia = Utils.WarpMatrix(m_rot, m_inertia0);
        m_inertiainv = Utils.WarpMatrix(m_rot, m_inertia0inv);

    }

    public int GetNumDoFs()
    {
        return 6;
    }

    public void GetPosition(VectorXD position)
    {
        position.SetSubVector(index, 3, Utils.ToVectorXD(m_pos));

        //Transform the quaternion to axis-angle
        Vector3 axisangle = Utils.ToAxisAngle(m_rot);
        position.SetSubVector(index + 3, 3, Utils.ToVectorXD(axisangle));
    }

    public void SetPosition(VectorXD position)
    {
        m_pos = Utils.ToVector3(position.SubVector(index, 3));

        //Transform the axis-angle to quaternion
        Vector3 axisangle = Utils.ToVector3(position.SubVector(index + 3, 3));
        m_rot = Utils.ToQuaternion(axisangle);

        m_inertia = Utils.WarpMatrix(m_rot, m_inertia0);
        m_inertiainv = Utils.WarpMatrix(m_rot, m_inertia0inv);
    }

    public void AdvanceIncrementalPosition(VectorXD position)
    {
        m_pos += Utils.ToVector3(position.SubVector(index, 3));

        //Transform the axis-angle to quaternion
        Vector3 axisangle = Utils.ToVector3(position.SubVector(index + 3, 3));
        m_rot = Utils.ToQuaternion(axisangle) * m_rot;

        m_inertia = Utils.WarpMatrix(m_rot, m_inertia0);
        m_inertiainv = Utils.WarpMatrix(m_rot, m_inertia0inv);
    }

    public void GetVelocity(VectorXD velocity)
    {
        velocity.SetSubVector(index, 3, Utils.ToVectorXD(m_vel));
        velocity.SetSubVector(index + 3, 3, Utils.ToVectorXD(m_omega));
    }

    public void SetVelocity(VectorXD velocity)
    {
        m_vel = Utils.ToVector3(velocity.SubVector(index, 3));
        m_omega = Utils.ToVector3(velocity.SubVector(index + 3, 3));
    }

    public void GetForce(VectorXD force)
    {
        // TO BE COMPLETED
        /*
          
         * IMPLEMENTACIÓN PROPIA
         * 
         * NO LA ELIMINO DEBIDO A QUE AGREGO LA QUE EL PROFESOR HA SUBIDO AL ARCHIVO "SolidoRigidoSolucion".
         *
        
        Vector3 fGravedad = Mass * Manager.Gravity;
        Vector3 fAmortiguamiento = -Damping * m_vel;
        Vector3 fTotal = fGravedad + fAmortiguamiento;
        
        force[index] += fTotal.x;
        force[index + 1] += fTotal.y;
        force[index + 2] += fTotal.z;
        */

        //Add gravity and damping to the force
        Vector3 Force = Mass * Manager.Gravity - Damping * Mass * m_vel;
        force.SetSubVector(index, 3, Utils.ToVectorXD(Force));

        //Add quadratic velocity vector and damping to the torque
        Vector3 Torque = -Vector3.Cross(m_omega, Utils.ToVector3(m_inertia * Utils.ToVectorXD(m_omega)))
            - Damping * Utils.ToVector3(m_inertia * Utils.ToVectorXD(m_omega));
        force.SetSubVector(index + 3, 3, Utils.ToVectorXD(Torque));

    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // TO BE COMPLETED
        //Derivative of damping on the linear force
        dFdv.SetSubMatrix(index, index, dFdv.SubMatrix(index, 3, index, 3)
            - Damping * Mass * DenseMatrixXD.CreateIdentity(3));

        //Derivative of damping on the torque and of the quadratic velocity vector
        // T = skew(M * w) * w = - skew(w) * (M * w)
        // dTdw = skew(M * w) - skew(w) * M
        dFdv.SetSubMatrix(index + 3, index + 3, dFdv.SubMatrix(index + 3, 3, index + 3, 3) - Damping * m_inertia
            + Utils.Skew(Utils.ToVector3(m_inertia * Utils.ToVectorXD(m_omega))) - Utils.Skew(m_omega) * m_inertia);
    }

    public void GetMass(MatrixXD mass)
    {
        mass.SetSubMatrix(index, index, mass.SubMatrix(index, 3, index, 3)
            + Mass * DenseMatrixXD.CreateIdentity(3));
        mass.SetSubMatrix(index + 3, index + 3, mass.SubMatrix(index + 3, 3, index + 3, 3)
            + m_inertia);
    }

    public void GetMassInverse(MatrixXD massInv)
    {
        massInv.SetSubMatrix(index, index, massInv.SubMatrix(index, 3, index, 3)
            + 1.0f / Mass * DenseMatrixXD.CreateIdentity(3));
        massInv.SetSubMatrix(index + 3, index + 3, massInv.SubMatrix(index + 3, 3, index + 3, 3)
            + m_inertiainv);
    }

    public void FixVector(VectorXD v)
    {
    }

    public void FixMatrix(MatrixXD M)
    {
    }

    #endregion

    #region OtherMethods

    public Vector3 PointGlobalToLocal(Vector3 p)
    {
        return Quaternion.Inverse(this.m_rot) * (p - this.m_pos);
    }

    public Vector3 PointLocalToGlobal(Vector3 p)
    {
        return this.m_pos + this.m_rot * p;
    }

    public Vector3 VecLocalToGlobal(Vector3 v)
    {
        return this.m_rot * v;
    }

    #endregion

}
