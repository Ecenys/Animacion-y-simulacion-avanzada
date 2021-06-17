using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

public class Spring : MonoBehaviour {

    #region InEditorVariables

    public float Stiffness;
    public float Damping;
    public Node nodeA;
    public Node nodeB;

    #endregion

    public enum SpringType { Stretch, Bend };
    public SpringType springType;

    public float Length0;
    public float Length;
    public Vector3 dir;

    private PhysicsManager Manager;

    public Spring(Node a, Node b, SpringType s)
    {
        nodeA = a;
        nodeB = b;
        springType = s;
    }

    // Update is called once per frame
    void Update () {
	}

    // Use this for initialization
    public void Initialize(float stiffness, float damping, PhysicsManager m)
    {
        // TO BE COMPLETED
        Stiffness = stiffness;
        Damping = damping;
        Manager = m;

        UpdateState();
        Length0 = Length;
    }

    // Update spring state
    public void UpdateState()
    {
        dir = nodeA.Pos - nodeB.Pos;
        Length = dir.magnitude;
        dir = (1.0f / Length) * dir;
    }

    // Get Force
    public void GetForce(VectorXD force)
    {
        // TO BE COMPLETED
        Vector3 totalForce = -Stiffness * (Length - Length0) * getUnitVector();
        //Fuerza de amortiguamiento
        totalForce -= (Damping * Stiffness) * Vector3.Dot(getUnitVector(), (nodeA.Vel - nodeB.Vel)) * getUnitVector();

        force[nodeA.index] += totalForce.x;
        force[nodeB.index] -= totalForce.x;

        force[nodeA.index + 1] += totalForce.y;
        force[nodeB.index + 1] -= totalForce.y;

        force[nodeA.index + 2] += totalForce.z;
        force[nodeB.index + 2] -= totalForce.z;
    }
    public Vector3 getUnitVector()
    {
        return (nodeA.Pos - nodeB.Pos) / Length;
    }

    // Get Force Jacobian
    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // TO BE COMPLETED
        Vector3 Felastica0 = -Stiffness * (Length - Length0) * dir;
        Vector3 Felastica1 = -Felastica0;


        Vector3 J1X = (1 / Length) * (-Stiffness * (Length - Length0) - Stiffness * (nodeA.Pos.x - nodeB.Pos.x) + Felastica0.x) * (Stiffness * (Length - Length0)) * -dir;
        Vector3 J1Y = (1 / Length) * (-Stiffness * (Length - Length0) - Stiffness * (nodeA.Pos.y - nodeB.Pos.y) + Felastica0.x) * (Stiffness * (Length - Length0)) * -dir;
        Vector3 J1Z = (1 / Length) * (-Stiffness * (Length - Length0) - Stiffness * (nodeA.Pos.z - nodeB.Pos.z) + Felastica0.x) * (Stiffness * (Length - Length0)) * -dir;

        Vector3 J0X = (1 / Length) * (-Stiffness * (Length - Length0) - Stiffness * (nodeA.Pos.x - nodeB.Pos.x) + Felastica0.x) * (Stiffness * (Length - Length0)) * -dir;
        Vector3 J0Y = (1 / Length) * (-Stiffness * (Length - Length0) - Stiffness * (nodeA.Pos.y - nodeB.Pos.y) + Felastica0.x) * (Stiffness * (Length - Length0)) * -dir;
        Vector3 J0Z = (1 / Length) * (-Stiffness * (Length - Length0) - Stiffness * (nodeA.Pos.z - nodeB.Pos.z) + Felastica0.x) * (Stiffness * (Length - Length0)) * -dir;

        dFdx[nodeB.index, nodeB.index] += J1X.x;
        dFdx[nodeB.index + 1, nodeB.index + 1] += J1X.x;
        dFdx[nodeB.index + 2, nodeB.index + 2] += J1X.x;
        dFdx[nodeA.index, nodeA.index] += J1X.x;
        dFdx[nodeA.index + 1, nodeA.index + 1] += J1X.x;
        dFdx[nodeA.index + 2, nodeA.index + 2] += J1X.x;

        //Restamos la amortiguación.
        for (int i = 0; i < dFdx.RowCount; i++)
        {
            dFdx[i, i] -= Damping;
        }

    }

}
