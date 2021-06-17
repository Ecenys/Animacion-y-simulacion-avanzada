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
    public float Pos;

    // Use this for initialization
    void Start () {
        this.nodeA.Start();
        this.nodeB.Start();
        this.Length0 = this.Length = Mathf.Abs(this.nodeA.Pos - this.nodeB.Pos);
        this.Pos = 0.5f * (this.nodeA.Pos + this.nodeB.Pos);
    }

    // Update is called once per frame
    void Update () {

        this.Length = Mathf.Abs(this.nodeA.Pos - this.nodeB.Pos);
        this.Pos = 0.5f * (this.nodeA.Pos + this.nodeB.Pos);

        this.transform.position = new Vector3(this.transform.position.x, this.Pos, this.transform.position.z);
        //The default length of a cylinder in Unity is 2.0
        this.transform.localScale = new Vector3(this.transform.localScale.x, this.Length / 2.0f, this.transform.localScale.z);
	}
}
