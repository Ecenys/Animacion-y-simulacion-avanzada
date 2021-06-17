using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic mass-spring model component which can be dropped onto
/// a game object and configured so that the set of nodes and
/// edges behave as a mass-spring model.
/// </summary>
public class MassSpring : MonoBehaviour, ISimulable
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public MassSpring()
    {
        Manager = null;
    }

    #region EditorVariables

    public List<Node> Nodes;
    public List<Spring> Springs;

    public float Mass;
    public float StiffnessStretch;
    public float StiffnessBend;
    public float DampingAlpha;
    public float DampingBeta;

    #endregion

    #region OtherVariables
    private PhysicsManager Manager;

    Vector3[] vertices;
    Mesh mesh;

    private int index;
    #endregion

    #region MonoBehaviour

    public void Awake()
    {
        mesh = this.GetComponent<MeshFilter>().mesh;
        vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        // TO BE COMPLETED
        for(int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = gameObject.transform.TransformPoint(vertices[i]);
        }

        Nodes = new List<Node>();
        Springs = new List<Spring>();

        foreach (Vector3 vertex in vertices)
            Nodes.Add(new Node(vertex));

        Dictionary<Spring, Node> springDictionary = new Dictionary<Spring, Node>(new SpringComparer());

        for (int i = 0; i < triangles.Length; i += 3)
        {
            Node a = Nodes[triangles[i]];
            Node b = Nodes[triangles[i + 1]];
            Node c = Nodes[triangles[i + 2]];

            Spring ab = new Spring(a, b, Spring.SpringType.Stretch);
            Spring bc = new Spring(b, c, Spring.SpringType.Stretch);
            Spring ca = new Spring(b, a, Spring.SpringType.Stretch);

            if (!springDictionary.ContainsKey(ab))
            {
                Springs.Add(ab);
                springDictionary.Add(ab, c);
            }
            else
                Springs.Add(new Spring(c, springDictionary[ab], Spring.SpringType.Bend));
            
            if (!springDictionary.ContainsKey(bc))
            {
                Springs.Add(bc);
                springDictionary.Add(bc, a);
            }
            else
                Springs.Add(new Spring(a, springDictionary[bc], Spring.SpringType.Bend));

            if (!springDictionary.ContainsKey(ca))
            {
                Springs.Add(ca);
                springDictionary.Add(ca, b);
            }
            else
                Springs.Add(new Spring(b, springDictionary[ca], Spring.SpringType.Bend));
        }
    }

    public void Update()
    {
        // TO BE COMPLETED
        for (int i = 0; i < Nodes.Count; i++)
        {
            vertices[i] = transform.InverseTransformPoint(Nodes[i].Pos);
        }
        mesh.vertices = vertices;
    }

    public void FixedUpdate()
    {
        // TO BE COMPLETED
    }
    #endregion

    #region ISimulable

    public void Initialize(int ind, PhysicsManager m, List<Fixer> fixers)
    {
        float nodeMass = Mass / Nodes.Count;

        // TO BE COMPLETED
        index = ind;
        Manager = m;

        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].Initialize(index + 3 * i, nodeMass, DampingAlpha, Manager);

        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].Initialize(StiffnessBend, DampingBeta, Manager);

        for (int j = 0; j < fixers.Count; j++)
        {
            for (int i = 0; i < Nodes.Count; i++)
                    {
                        Nodes[i].Fixed = fixers[0].IsInside(Nodes[i].Pos);
                    }
        }
        
    }

    public int GetNumDoFs()
    {
        return 3 * Nodes.Count;
    }

    public void GetPosition(VectorXD position)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetPosition(position);
    }

    public void SetPosition(VectorXD position)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].SetPosition(position);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].UpdateState();
    }

    public void GetVelocity(VectorXD velocity)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetVelocity(velocity);
    }

    public void SetVelocity(VectorXD velocity)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].SetVelocity(velocity);
    }

    public void GetForce(VectorXD force)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetForce(force);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].GetForce(force);
    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetForceJacobian(dFdx, dFdv);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].GetForceJacobian(dFdx, dFdv);
    }

    public void GetMass(MatrixXD mass)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetMass(mass);
    }

    public void GetMassInverse(MatrixXD massInv)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetMassInverse(massInv);
    }

    public void FixVector(VectorXD v)
    {
        for (int i = 0; i < Nodes.Count; i++)
        {
            Nodes[i].FixVector(v);
        }
    }

    public void FixMatrix(MatrixXD M)
    {
        for (int i = 0; i < Nodes.Count; i++)
        {
            Nodes[i].FixMatrix(M);
        }
    }

    #endregion

    #region OtherMethods

    #endregion

}
