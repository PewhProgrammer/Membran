using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Text2Speech : Photon.PunBehaviour {
    
    private string result = "";
    private string hypothesis = "";

	// Use this for initialization
	void Start () {
        //PhotonNetwork.ConnectUsingSettings("v1.0");
	}

    [PunRPC]
    public void sendString(string result, string hypothesis)
    {
        this.result = result;
        WindowsVoice.theVoice.speak(result);
        this.hypothesis = hypothesis;
        WindowsVoice.theVoice.speak(hypothesis);
    }

    override
    public void OnConnectedToMaster()
    {
        RoomOptions rOpts = new RoomOptions() { IsVisible = false, MaxPlayers = 5 };
        PhotonNetwork.JoinOrCreateRoom("Speech", rOpts, TypedLobby.Default);
    }
	
	// Update is called once per frame
	void Update () {
		
	}

    void OnGUI()
    {
        GUI.Label(new Rect(20, 40, 1000, 20), "result");
        GUI.Label(new Rect(20, 60, 1000, 20), result);
        GUI.Label(new Rect(20, 80, 1000, 20), "hypothesis");
        GUI.Label(new Rect(20, 100, 1000, 20), hypothesis);
        GUI.Label(new Rect(20, 120, 1000, 20), PhotonNetwork.connectionStateDetailed.ToString());
    }
}
