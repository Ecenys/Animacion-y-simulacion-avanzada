using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node : MonoBehaviour {

    #region InEditorVariables

    public float Mass;

    #endregion

    public float Pos;
    public float Vel;

	// Use this for initialization
	public void Start () {

        this.Pos = this.transform.position.y;

	}
	
	// Update is called once per frame
	void Update () {

        this.transform.position = new Vector3(this.transform.position.x, this.Pos, this.transform.position.z);
	}
}
