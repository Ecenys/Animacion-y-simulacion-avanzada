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
    public Node nodeA;
    public Node nodeB;

    #endregion

    public float Length0;
    public float Length;

    private PhysicsManager Manager;

    // Update is called once per frame
    void Update () {

        Vector3 yaxis = new Vector3(0.0f, 1.0f, 0.0f);
        Vector3 dir = nodeA.Pos - nodeB.Pos;
        dir.Normalize();

        transform.position = 0.5f * (nodeA.Pos + nodeB.Pos);
        //The default length of a cylinder in Unity is 2.0
        transform.localScale = new Vector3(transform.localScale.x, Length / 2.0f, transform.localScale.z);
        transform.rotation = Quaternion.FromToRotation(yaxis, dir);
	}

    // Use this for initialization
    public void Initialize(PhysicsManager m)
    {
        Manager = m;

        UpdateState();
        Length0 = Length;
    }

    // Update spring state
    public void UpdateState()
    {
        Length = (nodeA.Pos - nodeB.Pos).magnitude;
    }

    // Get Force
    public void GetForce(VectorXD force)
    {
        // TO BE COMPLETED //
        Vector3 Force = -Stiffness * (Length - Length0) * (nodeA.Pos - nodeB.Pos) / Length;
        
        force[nodeA.index] += Force.x;
        force[nodeB.index] -= Force.x;

        force[nodeA.index + 1] += Force.y;
        force[nodeB.index + 1] -= Force.y;

        force[nodeA.index + 2] += Force.z;
        force[nodeB.index + 2] -= Force.z;

    }

    // Get Force Jacobian
    public void GetForceJacobian(MatrixXD dFdx)
    {
        // TO BE COMPLETED //
        Vector3 Felastica0 = -Stiffness * (Length - Length0) * ((nodeA.Pos - nodeB.Pos) / Length);
        Vector3 Felastica1 = -Stiffness * (Length - Length0) * ((nodeB.Pos - nodeA.Pos) / Length);

        //Calculamos la Jacobiana de la Felastica para cada dirección de cada nodo
        Vector3 J1X = (1 / Length) * (-Stiffness * (Length - Length0) - Stiffness * (nodeB.Pos.x - nodeA.Pos.x) + Felastica0.x) * (Stiffness * (Length - Length0) * ((nodeA.Pos - nodeB.Pos) / Length));
        Vector3 J1Y = (1 / Length) * (-Stiffness * (Length - Length0) - Stiffness * (nodeB.Pos.y - nodeA.Pos.y) + Felastica0.y) * (Stiffness * (Length - Length0) * ((nodeA.Pos - nodeB.Pos) / Length));
        Vector3 J1Z = (1 / Length) * (-Stiffness * (Length - Length0) - Stiffness * (nodeB.Pos.z - nodeA.Pos.z) + Felastica0.z) * (Stiffness * (Length - Length0) * ((nodeA.Pos - nodeB.Pos) / Length));
        Vector3 J0X = (1 / Length) * (-Stiffness * (Length - Length0) - Stiffness * (nodeA.Pos.x - nodeB.Pos.x) + Felastica1.x) * (Stiffness * (Length - Length0) * ((nodeB.Pos - nodeA.Pos) / Length));
        Vector3 J0Y = (1 / Length) * (-Stiffness * (Length - Length0) - Stiffness * (nodeA.Pos.y - nodeB.Pos.y) + Felastica1.y) * (Stiffness * (Length - Length0) * ((nodeB.Pos - nodeA.Pos) / Length));
        Vector3 J0Z = (1 / Length) * (-Stiffness * (Length - Length0) - Stiffness * (nodeA.Pos.z - nodeB.Pos.z) + Felastica1.z) * (Stiffness * (Length - Length0) * ((nodeB.Pos - nodeA.Pos) / Length));

        //Asignamos los valores de la Jacobiana en su lugar corredpondiente en la matriz
        dFdx[nodeA.index, nodeA.index] += J1X.x;
        dFdx[nodeA.index + 1, nodeA.index + 1] += J1Y.y;
        dFdx[nodeA.index + 2, nodeA.index + 2] += J1Z.z;
        dFdx[nodeB.index, nodeB.index] += J0X.x;
        dFdx[nodeB.index + 1, nodeB.index + 1] += J0Y.y;
        dFdx[nodeB.index + 2, nodeB.index + 2] += J0Z.z;
    }

}
