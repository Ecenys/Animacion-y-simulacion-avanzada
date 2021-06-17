using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic point constraint between two rigid bodies.
/// </summary>
public class PointConstraint : MonoBehaviour, IConstraint
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public PointConstraint()
    {
        Manager = null;
    }

    #region EditorVariables

    public float Stiffness;

    public RigidBody bodyA;
    public RigidBody bodyB;

    #endregion

    #region OtherVariables

    int index;
    private PhysicsManager Manager;

    protected Vector3 pointA;
    protected Vector3 pointB;

    #endregion

    #region MonoBehaviour

    // Update is called once per frame
    void Update()
    {
        // Compute the average position
        Vector3 posA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 posB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;
        Vector3 pos = 0.5f * (posA + posB);

        // Apply the position
        Transform xform = GetComponent<Transform>();
        xform.position = pos;
    }

    #endregion

    #region IConstraint

    public void Initialize(int ind, PhysicsManager m)
    {
        index = ind;
        Manager = m;

        // Initialize local positions. We assume that the object is connected to a Sphere mesh.
        Transform xform = GetComponent<Transform>();
        if (xform == null)
        {
            System.Console.WriteLine("[ERROR] Couldn't find any transform to the constraint");
        }
        else
        {
            System.Console.WriteLine("[TRACE] Succesfully found transform connected to the constraint");
        }

        // Initialize kinematics
        Vector3 pos = xform.position;

        // Local positions on objects
        pointA = (bodyA != null) ? bodyA.PointGlobalToLocal(pos) : pos;
        pointB = (bodyB != null) ? bodyB.PointGlobalToLocal(pos) : pos;

    }

    public int GetNumConstraints()
    {
        return 3;
    }

    public void GetConstraints(VectorXD c)
    {
        // TO BE COMPLETED
        Vector3 restriccion = pointA - pointB;

        c[index] = restriccion.x;
        c[index + 1] = restriccion.y;
        c[index + 2] = restriccion.z;
    }

    public void GetConstraintJacobian(MatrixXD dcdx)
    {
        // TO BE COMPLETED
        //Creamos la matriz Identidad para utilizarla en la matriz Jacobiana
        MatrixXD I = DenseMatrixXD.CreateIdentity(3);

        //Si existe un Rigidoby en la parte A del contrain
        if (bodyA != null)
        {
            dcdx.SetSubMatrix(index, bodyA.index, I);
            dcdx.SetSubMatrix(index, bodyA.index + 3, -Utils.Skew(bodyA.VecLocalToGlobal(pointA)));
        }

        if (bodyB != null)
        {
            dcdx.SetSubMatrix(index, bodyB.index, -I);
            dcdx.SetSubMatrix(index, bodyB.index + 3, Utils.Skew(bodyB.VecLocalToGlobal(pointB)));
        }
    }

    public void GetForce(VectorXD force)
    {

        // TO BE COMPLETED
        /*
         
         * IMPLEMENTACIÓN PROPIA *NO FUNCIONA CORRECTAMENTE LA MODIFICACIÓN DEL TORQUE*
         * 
         * NO LA ELIMINO DEBIDO A QUE AGREGO LA QUE EL PROFESOR HA SUBIDO AL ARCHIVO "SolidoRigidoSolucion".
         * 
        
        Vector3 pA, pB;
        //Vector3 torque;
        if (bodyA)
            pA = bodyA.PointLocalToGlobal(pointA);
        else
            pA = pointA;

        if (bodyB)
            pB = bodyB.PointLocalToGlobal(pointB);
        else
            pB = pointB;

        if (bodyA)
        {
            Vector3 fA = Stiffness * (pB - pA);
            force[bodyA.index] += fA.x;
            force[bodyA.index + 1] += fA.y;
            force[bodyA.index + 2] += fA.z;

            //torque = Vector3.Cross(bodyA.VecLocalToGlobal(pointA), fA);
            //Debug.Log("Torque A del nodo " + index + ": " + torque);
            //force[bodyA.index] += torque.x;
            //force[bodyA.index + 1] += torque.y;
            //force[bodyA.index + 2] += torque.z;
        }

        if (bodyB)
        {
            Vector3 fB = Stiffness * (pA - pB);
            force[bodyB.index] += fB.x;
            force[bodyB.index + 1] += fB.y;
            force[bodyB.index + 2] += fB.z;

            //torque = Vector3.Cross(bodyB.VecLocalToGlobal(pointB), fB);
            //Debug.Log("Torque B del nodo " + index + ": " + torque);
            //force[bodyB.index] += torque.x;
            //force[bodyB.index + 1] += torque.y;
            //force[bodyB.index + 2] += torque.z;
        }
        */


        Vector3 posA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 posB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;

        Vector3 f = -Stiffness * (posA - posB);

        if (bodyA != null)
        {
            force.SetSubVector(bodyA.index, 3, force.SubVector(bodyA.index, 3) + Utils.ToVectorXD(f));
            force.SetSubVector(bodyA.index + 3, 3, force.SubVector(bodyA.index + 3, 3)
                + Utils.ToVectorXD(Vector3.Cross(posA - bodyA.m_pos, f)));
        }
        if (bodyB != null)
        {
            force.SetSubVector(bodyB.index, 3, force.SubVector(bodyB.index, 3) + Utils.ToVectorXD(-f));
            force.SetSubVector(bodyB.index + 3, 3, force.SubVector(bodyB.index + 3, 3)
                + Utils.ToVectorXD(Vector3.Cross(posB - bodyB.m_pos, -f)));
        }
    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // TO BE COMPLETED
    }

    #endregion

}
