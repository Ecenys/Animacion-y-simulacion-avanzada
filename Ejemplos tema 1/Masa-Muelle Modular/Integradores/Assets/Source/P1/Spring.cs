using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spring : MonoBehaviour {

    #region InEditorVariables

    public float Stiffness;
    public Node nodeA;
    public Node nodeB;

    #endregion

    public float Length0;
    public float Length;
    public Vector3 Pos;

    public PhysicsManager Manager;

    // Update is called once per frame
    void Update () {

        transform.position = Pos;
        //The default length of a cylinder in Unity is 2.0
        this.transform.localScale = new Vector3(this.transform.localScale.x, this.Length / 2.0f, this.transform.localScale.z);
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
        Pos = 0.5f * (nodeA.Pos + nodeB.Pos);
    }

    // Computer force and add to the nodes
    public void ComputeForce()
    {
        Vector3 Force = -Stiffness * (Length - Length0) * (nodeA.Pos - nodeB.Pos) / Length;
        nodeA.Force += Force;
        nodeB.Force -= Force;
    }
}
