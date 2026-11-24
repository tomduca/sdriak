#pragma once
#include <imgui.h>

// Sideband toggle for an SDR on a transceiver's IF.
//
// On an IF panadapter the spectrum is often inverted relative to the radio:
// the radio's USB shows up as LSB on the SDR and vice versa. Flipping the
// sideband on the radio also moves its signal to a different spot in the IF.
// This toggle sits in the top bar and does both in one click: it shows the
// radio's sideband (USB/LSB) and, when pressed, tunes the selected VFO to the
// IF frequency for that sideband and sets the Radio module to the opposite
// (SDR-side) sideband.
//
// Off by default; enabled, with its two IF frequencies, from the Display menu.
namespace ifsideband {
    void init();

    bool isEnabled();

    // Top bar button, sized like the toolbar's icon buttons. Draws nothing
    // when the feature is disabled.
    void drawButton(const ImVec2& btnSize, int padding);

    // Settings block for the Display menu.
    void drawSettings();
}
