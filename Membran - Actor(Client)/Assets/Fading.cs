using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Fading : Photon.PunBehaviour {

    //object initial visible?
    public bool visible;

    //how fast fade?
    private float step = 0.01f;

    public float maxOpaque = 1.0f;
    public float minOpaque = 0.0f;
    
    private SkinnedMeshRenderer rend;

    private Color[] colors;
    private Color[] emissions;

    [PunRPC]
    public void toggleFading()
    {
        
        Debug.Log("incoming fading...");
        visible = !visible;
        if (visible)
        {
            rend.enabled = true;
        }

        if (visible) {
            if (rend.material.color.a != 1.0f) {
                increaseOpaque();
            }
        } else {
            decreaseOpaque();
            if (rend.material.color.a == 0.0f) {
                rend.enabled = false;
            }
        }
    }

    // Use this for initialization
    void Start () {
        rend = GetComponent<SkinnedMeshRenderer>();
        
        rend.enabled = visible;
        float x = visible ? maxOpaque : minOpaque;

        colors = new Color[rend.materials.Length];
        emissions = new Color[rend.materials.Length];

        for (int i = 0; i != rend.materials.Length; i++) {
            colors[i] = rend.materials[i].GetColor("_Color");
            emissions[i] = rend.materials[i].GetColor("_EmissionColor");
        }

        for (int i = 0; i != rend.materials.Length; i++) {
            float r = colors[i].r;
            float g = colors[i].g;
            float b = colors[i].b;
            rend.materials[i].SetColor("_Color", new Color(r, g, b, x));
            rend.materials[i].SetColor("_EmissionColor", emissions[i] * x);
        }


    }
	
	// Update is called once per frame
	void Update () {
		if (Input.GetKeyDown("d")) {
            this.photonView.RPC("toggleFadding", PhotonTargets.Others);
            visible = !visible;
            if (visible) {
                rend.enabled = true;
            }
        }

        if (visible) {
            if (rend.material.color.a != 1.0f) {
                increaseOpaque();
            }
        } else {
            decreaseOpaque();
            if (rend.material.color.a == 0.0f) {
                rend.enabled = false;
            }
        }
    }
  private void increaseOpaque() {
        float current = Math.Min(rend.material.color.a + step, maxOpaque);
        for (int i = 0; i != rend.materials.Length; i++) {
            float r = colors[i].r;
            float g = colors[i].g;
            float b = colors[i].b;
            rend.materials[i].SetColor("_Color", new Color(r, g, b, current));
            rend.materials[i].SetColor("_EmissionColor", emissions[i] * current);
        }
    }

    private void decreaseOpaque() {
        float current = Math.Max(rend.material.color.a - step, minOpaque);
        for (int i = 0; i != rend.materials.Length; i++) {
            float r = colors[i].r;
            float g = colors[i].g;
            float b = colors[i].b;
            rend.materials[i].SetColor("_Color", new Color(r, g, b, current));
            rend.materials[i].SetColor("_EmissionColor", emissions[i] * current);
        }
    }

  
}
