using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

public class Node : MonoBehaviour {

    #region InEditorVariables

    public float Mass;
    public float Damping;
    public bool Fixed;

    #endregion

    public int index;

    public Vector3 Pos;
    public Vector3 Vel;

    private PhysicsManager Manager;

    public Node(Vector3 p)
    {
        Pos = p;
        Vel = Vector3.zero;
        Fixed = false;
    }

    // Update is called once per frame
    void Update () {
	}

    // Use this for initialization
    public void Initialize(int ind, float mass, float damping, PhysicsManager m)
    {
        // TO BE COMPLETED
        index = ind;
        Mass = mass;
        Damping = damping;
        Manager = m;
    }

    // Get Force
    public void GetForce(VectorXD force)
    {
        // TO BE COMPLETED
        force[index] = Mass * Manager.Gravity.x;
        force[index + 1] = Mass * Manager.Gravity.y;
        force[index + 2] = Mass * Manager.Gravity.z;
    }

    // Get Force Jacobian
    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // TO BE COMPLETED
        dFdx[index, index] = Mass * Manager.Gravity.x;
        dFdx[index + 1, index + 1] = Mass * Manager.Gravity.y;
        dFdx[index + 2, index + 2] = Mass * Manager.Gravity.z;
    }

    public void GetPosition(VectorXD pos)
    {
        pos[index] = Pos.x;
        pos[index + 1] = Pos.y;
        pos[index + 2] = Pos.z;
    }

    public void SetPosition(VectorXD pos)
    {
        Pos = new Vector3((float)pos[index], (float)pos[index + 1], (float)pos[index + 2]);
    }

    public void GetVelocity(VectorXD vel)
    {
        vel[index] = Vel.x;
        vel[index + 1] = Vel.y;
        vel[index + 2] = Vel.z;
    }

    public void SetVelocity(VectorXD vel)
    {
        Vel = new Vector3((float)vel[index], (float)vel[index + 1], (float)vel[index + 2]);
    }

    public void GetMass(MatrixXD mass)
    {
        mass[index, index] = Mass;
        mass[index+1, index+1] = Mass;
        mass[index+2, index+2] = Mass;
    }

    public void GetMassInverse(MatrixXD massInv)
    {
        massInv[index, index] = 1.0 / Mass;
        massInv[index + 1, index + 1] = 1.0 / Mass;
        massInv[index + 2, index + 2] = 1.0 / Mass;
    }

    public void FixVector(VectorXD v)
    {
        if(Fixed)
        {
            v[index] = 0.0;
            v[index+1] = 0.0;
            v[index+2] = 0.0;
        }
    }

    public void FixMatrix(MatrixXD M)
    {
        if (Fixed)
        {
            for (int i = 0; i < M.RowCount; i++)
            {
                M[index, i] = 0.0;
                M[index+1, i] = 0.0;
                M[index+2, i] = 0.0;
                M[i, index] = 0.0;
                M[i, index+1] = 0.0;
                M[i, index+2] = 0.0;
            }
            M[index, index] = 1.0;
            M[index+1, index+1] = 1.0;
            M[index+2, index+2] = 1.0;
        }
    }

}
