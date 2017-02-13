using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Windows.Speech;

public class Speech2Text : Photon.PunBehaviour {

    DictationRecognizer dictationRecognizer;
    static string result = "";

    private bool running = false;
    private bool stop = false;

    private string t = "";

    // Use this for initialization
    void Start() {

        PhotonNetwork.ConnectUsingSettings("v1.0");

        dictationRecognizer = new DictationRecognizer();

        dictationRecognizer.DictationResult += onDictationResult;
        dictationRecognizer.DictationComplete += onDictationComplete;
        dictationRecognizer.DictationError += onDictationError;
    }

    override
    public void OnConnectedToMaster() {
        RoomOptions rOpts = new RoomOptions() { IsVisible = false,
        MaxPlayers = 5};

        PhotonNetwork.JoinOrCreateRoom("Actor", rOpts, TypedLobby.Default);
    } 

    void onDictationResult(string text, ConfidenceLevel confidence) {
        result = text;
        t = text;
        Debug.LogFormat("Dictation result: " + text);

        if (text.ToLower().Equals("stopp")) {
            photonView.RPC("toggleFade", PhotonTargets.Others);
        }
    }

    void onDictationComplete(DictationCompletionCause cause) {
        if (cause != DictationCompletionCause.Complete)
            Debug.LogErrorFormat("Dictation completed unsuccessfully: {0}.", cause);

        if (cause == DictationCompletionCause.TimeoutExceeded) {
            dictationRecognizer.Stop();
            dictationRecognizer.Start();
        }
    }

    void onDictationError(string error, int hresult) {
        Debug.LogErrorFormat("Dictation error: {0}; HResult = {1}.", error, hresult);
    }
	
	// Update is called once per frame
	void Update () { 
        //activate deactivate speech recognition
        if (Input.GetKeyDown("s"))
        {
            if (running) {
                dictationRecognizer.Stop();
                result = "";  
            } else {
                dictationRecognizer.Start();
            }
            running = !running;
        }

        this.photonView.RPC("sendString", PhotonTargets.Others, getResult());
    }

    [PunRPC]
    public void sendString(string r) {
    }

    [PunRPC]
    public void toggleFade() {
    }

    private string getResult() {
        string temp = string.Copy(result);
        result = "";
        if (temp.ToLower().Equals("stopp")) {
            stop = !stop;
        }
        return temp;
    }

    void OnGUI() {
        GUI.Label(new Rect(20, 20, 1000, 20), PhotonNetwork.connectionStateDetailed.ToString());
        GUI.Label(new Rect(20, 40, 1000, 20), "Voice input activated: " + running.ToString());
        GUI.Label(new Rect(20, 60, 1000, 20), "HBK voice output activated: " + (!stop).ToString());
        GUI.Label(new Rect(20, 80, 1000, 20), "Input: " + t);
    }
}
