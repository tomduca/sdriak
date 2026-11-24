#include <gui/if_sideband.h>
#include <gui/gui.h>
#include <gui/style.h>
#include <gui/tuner.h>
#include <gui/widgets/toggle_style.h>
#include <core.h>
#include <radio_interface.h>
#include <string>

namespace ifsideband {
    namespace {
        bool enabled = false;
        double usbFreq = 8831500.0; // IF frequency while the radio is on USB
        double lsbFreq = 8828500.0; // IF frequency while the radio is on LSB
        bool radioUsb = true;       // last sideband chosen with the toggle

        bool isRadioVFO(const std::string& vfoName) {
            return !vfoName.empty() && core::modComManager.interfaceExists(vfoName)
                && core::modComManager.getModuleName(vfoName) == "radio";
        }

        // The radio's sideband as implied by the Radio module's mode (the SDR
        // sees it inverted). Falls back to the last toggle state when the mode
        // is neither USB nor LSB, or there is no radio VFO.
        bool currentRadioUsb() {
            const std::string vfoName = gui::waterfall.selectedVFO;
            if (!isRadioVFO(vfoName)) { return radioUsb; }
            int mode = -1;
            core::modComManager.callInterface(vfoName, RADIO_IFACE_CMD_GET_MODE, NULL, &mode);
            if (mode == RADIO_IFACE_MODE_LSB) { return true; }
            if (mode == RADIO_IFACE_MODE_USB) { return false; }
            return radioUsb;
        }

        void save() {
            auto conf = core::configManager.edit();
            conf.set("ifSidebandToggle", json({
                { "enabled", enabled },
                { "usbFreq", usbFreq },
                { "lsbFreq", lsbFreq },
                { "radioUsb", radioUsb },
            }));
        }

        void apply(bool usb) {
            radioUsb = usb;
            const std::string vfoName = gui::waterfall.selectedVFO;
            if (isRadioVFO(vfoName)) {
                // The SDR sees the IF inverted: radio USB is LSB here.
                int mode = usb ? RADIO_IFACE_MODE_LSB : RADIO_IFACE_MODE_USB;
                int current = -1;
                core::modComManager.callInterface(vfoName, RADIO_IFACE_CMD_GET_MODE, NULL, &current);
                // SET_MODE rebuilds the demodulator even for the same mode
                // (an audible click), so only send it on a change.
                if (current != mode) {
                    core::modComManager.callInterface(vfoName, RADIO_IFACE_CMD_SET_MODE, &mode, NULL);
                }
            }
            // Tune after the mode change, which may move the VFO's offsets.
            if (!vfoName.empty()) {
                tuner::tune(tuner::TUNER_MODE_NORMAL, vfoName, usb ? usbFreq : lsbFreq);
            }
            save();
        }
    }

    void init() {
        auto conf = core::configManager.read();
        json section = conf.value("ifSidebandToggle", json::object());
        if (!section.is_object()) { return; }
        auto num = [&](const char* key, double& out) {
            if (section.contains(key) && section[key].is_number()) { out = section[key].get<double>(); }
        };
        if (section.contains("enabled") && section["enabled"].is_boolean()) { enabled = section["enabled"]; }
        if (section.contains("radioUsb") && section["radioUsb"].is_boolean()) { radioUsb = section["radioUsb"]; }
        num("usbFreq", usbFreq);
        num("lsbFreq", lsbFreq);
    }

    bool isEnabled() { return enabled; }

    void drawButton(const ImVec2& btnSize, int padding) {
        if (!enabled) { return; }

        const bool usb = currentRadioUsb();
        const char* label = usb ? "USB" : "LSB";
        const ImVec2 size(btnSize.x * 1.6f + 2.0f * padding, btnSize.y + 2.0f * padding);

        style::SelectedToggleColors cols = style::selectedToggleColors();
        style::pushSelectedToggle(cols);
        bool clicked = ImGui::Button((std::string(label) + "##sdrpp_if_sideband").c_str(), size);
        style::drawSelectedToggleStroke(cols);
        style::popSelectedToggle();

#ifndef __ANDROID__
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Radio sideband: %s\nClick for %s (IF %.4f MHz, Radio module %s)",
                              label, usb ? "LSB" : "USB",
                              (usb ? lsbFreq : usbFreq) / 1e6, usb ? "USB" : "LSB");
        }
#endif
        if (clicked) { apply(!usb); }
    }

    void drawSettings() {
        if (ImGui::Checkbox("IF Sideband Toggle##_sdrpp_ifsb", &enabled)) {
            save();
        }
        if (!enabled) { return; }

        // Frequencies in MHz, like the band plan; stored in Hz.
        ImGui::Indent();
        double usbMhz = usbFreq / 1e6;
        double lsbMhz = lsbFreq / 1e6;
        ImGui::TextUnformatted("Radio USB at IF");
        ImGui::SameLine();
        ImGui::FillWidth();
        if (ImGui::InputDouble("##_sdrpp_ifsb_usb", &usbMhz, 0.0, 0.0, "%.4f MHz", ImGuiInputTextFlags_EnterReturnsTrue) && usbMhz > 0.0) {
            usbFreq = usbMhz * 1e6;
            save();
        }
        ImGui::TextUnformatted("Radio LSB at IF");
        ImGui::SameLine();
        ImGui::FillWidth();
        if (ImGui::InputDouble("##_sdrpp_ifsb_lsb", &lsbMhz, 0.0, 0.0, "%.4f MHz", ImGuiInputTextFlags_EnterReturnsTrue) && lsbMhz > 0.0) {
            lsbFreq = lsbMhz * 1e6;
            save();
        }
        ImGui::Unindent();
    }
}
