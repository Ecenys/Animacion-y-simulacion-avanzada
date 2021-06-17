using UnityEngine;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class PhysicsManager : MonoBehaviour 
{
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public PhysicsManager()
	{
		this.Paused = true;
		this.TimeStep = 0.01f;
		this.Gravity = new Vector3 (0.0f, -9.81f, 0.0f);
		this.IntegrationMethod = Integration.Explicit;
	}

	/// <summary>
	/// Integration method.
	/// </summary>
	public enum Integration
	{
		Explicit = 0,
		Symplectic = 1,
        Midpoint = 2,
        Verlet = 3,
        Implicit = 4,
	};

	#region InEditorVariables

	public bool Paused;
	public float TimeStep;
    public Vector3 Gravity;
	public Integration IntegrationMethod;
    public Node nodeHigh;
    public Node nodeLow;
    public Spring spring;

    #endregion

    #region OtherVariables
    private bool first;
    private float xOld;
    #endregion

    #region MonoBehaviour

    public void Start()
    {
        nodeLow.Initialize(this);
        nodeHigh.Initialize(this);
        spring.Initialize(this);
    }

    public void Update()
	{
		if (Input.GetKeyUp (KeyCode.P))
			this.Paused = !this.Paused;

    }

    public void FixedUpdate()
    {
        if (this.Paused)
            return; // Not simulating

        // Select integration method
        switch (this.IntegrationMethod)
        {
            case Integration.Explicit: this.stepExplicit(); break;
            case Integration.Symplectic: this.stepSymplectic(); break;
            case Integration.Midpoint: this.stepMidpoint(); break;
            case Integration.Verlet: this.stepVerlet(); break;
            case Integration.Implicit: this.stepImplicit(); break;
            default:
                throw new System.Exception("[ERROR] Should never happen!");
        }
    }

    #endregion

    /// <summary>
    /// Performs a simulation step in 1D using Explicit integration.
    /// </summary>
    private void stepExplicit()
	{
        nodeLow.ResetForce();
        nodeLow.ComputeForce();
        spring.ComputeForce();

        nodeLow.Pos += TimeStep * nodeLow.Vel;
        nodeLow.Vel += TimeStep * (1.0f / nodeLow.Mass) * nodeLow.Force;

        spring.UpdateState();
	}

	/// <summary>
	/// Performs a simulation step in 1D using Symplectic integration.
	/// </summary>
	private void stepSymplectic()
	{
        nodeLow.ResetForce();
        nodeLow.ComputeForce();
        spring.ComputeForce();

        nodeLow.Vel += TimeStep * (1.0f / nodeLow.Mass) * nodeLow.Force;
        nodeLow.Pos += TimeStep * nodeLow.Vel;

        spring.UpdateState();
    }

    /// <summary>
    /// Performs a simulation step in 3D using Midpoint integration.
    /// </summary>
    /// 
    Node middleNode;
    private void stepMidpoint()
    {
        nodeLow.ResetForce();
        nodeLow.ComputeForce();
        spring.ComputeForce();

        middleNode.Vel = nodeLow.Vel + nodeLow.Force * (TimeStep / 2);

        middleNode.ResetForce();
        middleNode.ComputeForce();
        spring.ComputeForce();

        nodeLow.Vel = nodeLow.Vel + middleNode.Force * TimeStep;

        spring.UpdateState();
    }

    /// <summary>
    /// Performs a simulation step in 3D using Verlet integration.
    /// </summary>
    private void stepVerlet()
    {
        if (first)
        {
            //Code
            first = false;
        }
        else
        {
            //Code
        }

        //Velocity
    }

    /// <summary>
    /// Performs a simulation step in 1D using Implicit integration.
    /// </summary>
    private void stepImplicit()
    {
    }

}
