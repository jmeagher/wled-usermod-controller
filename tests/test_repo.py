"""Repository consistency checks that a compile does not catch. Run: python3 -I -m unittest discover -s tests -v"""
import configparser
import json
import re
import shutil
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = (ROOT / "controller.cpp").read_text()
SHA = re.compile(r"\b[0-9a-f]{40}\b")


def pin_defaults():
    """CTL_PIN_* defaults from controller.cpp."""
    return {m.group(1): int(m.group(2)) for m in re.finditer(r"#define CTL_PIN_(\w+)\s+(\d+)", SRC)}


class LibraryManifest(unittest.TestCase):
    def setUp(self):
        self.lib = json.loads((ROOT / "library.json").read_text())

    def test_sources_listed_exist(self):
        # WLED requires libArchive false; srcFilter keeps tests/ and tools out of the firmware.
        self.assertIs(self.lib["build"]["libArchive"], False)
        files = [f[2:-1] for f in self.lib["build"]["srcFilter"] if f.startswith("+<")]
        self.assertEqual(files, ["controller.cpp"])
        for f in files:
            self.assertTrue((ROOT / f).is_file(), f)

    def test_dependencies_are_pinned(self):
        for name, ver in self.lib["dependencies"].items():
            self.assertRegex(ver, r"^\d+\.\d+\.\d+$", f"{name} must be an exact version, got {ver!r}")


class Sources(unittest.TestCase):
    def test_includes_resolve(self):
        for inc in re.findall(r'#include "([^"]+)"', SRC):
            if inc == "wled.h":      # provided by the WLED tree this usermod is built into
                continue
            self.assertTrue((ROOT / inc).is_file(), inc)

    def test_serial_only_through_debug_macro(self):
        # Default builds must not write to the USB serial port: it is WLED's Improv/Adalight channel.
        stray = [l for l in SRC.splitlines() if "Serial." in l and "CTL_LOG" not in l and "#define" not in l]
        self.assertEqual(stray, [])

    def test_config_keys_round_trip(self):
        """Every key written by addToConfig is read back by readFromConfig, and vice versa."""
        written = set(re.findall(r'top\[F\("([^"]+)"\)\]\s*=', SRC))
        read = set(re.findall(r'getJsonValue\(top\[F\("([^"]+)"\)\]', SRC))
        self.assertTrue(written)
        self.assertEqual(written, read)

    def test_pin_defaults_complete(self):
        self.assertEqual(sorted(pin_defaults()), sorted(["MOSI", "SCLK", "CS", "DC", "RST", "BL", "TSDA", "TSCL", "TRST", "TINT"]))
        pins = list(pin_defaults().values())
        self.assertEqual(len(pins), len(set(pins)), "two signals share a default GPIO")


class DocsMatchCode(unittest.TestCase):
    def test_readme_pin_table(self):
        p = pin_defaults()
        readme = (ROOT / "README.md").read_text()
        lcd = " / ".join(str(p[k]) for k in ("SCLK", "MOSI", "CS", "DC", "RST", "BL"))
        touch = " / ".join(str(p[k]) for k in ("TSDA", "TSCL", "TRST", "TINT"))
        self.assertIn(lcd, readme)
        self.assertIn(touch, readme)


class BuildFiles(unittest.TestCase):
    def test_sample_override(self):
        cp = configparser.ConfigParser(interpolation=None, strict=False)
        cp.read(ROOT / "platformio_override.sample.ini")
        sec = cp["env:ctl_esp32c6_8MB"]
        self.assertEqual(sec["extends"], "env:esp32c6dev_8MB")
        # WLED reads custom_usermods line by line: each entry on its own line.
        entries = [l.strip() for l in sec["custom_usermods"].splitlines() if l.strip()]
        self.assertIn("symlink://__USERMOD_PATH__", entries)
        # Without BTNPIN=-1 WLED claims GPIO 0 (battery sense on this board) as its own button.
        self.assertIn("-D BTNPIN=-1", sec["build_flags"])

    def test_install_script_keeps_btnpin(self):
        self.assertIn("-D BTNPIN=-1", (ROOT / "scripts" / "install.sh").read_text())

    def test_pinned_wled_commit_is_consistent(self):
        """The WLED commit is defined in scripts/dev-setup.sh and CI must use the same one. install.sh must not
        carry a second copy (it takes the pin from dev-setup.sh)."""
        pin = set(SHA.findall((ROOT / "scripts" / "dev-setup.sh").read_text()))
        self.assertEqual(len(pin), 1, f"dev-setup.sh should name exactly one WLED commit, found {pin}")
        self.assertEqual(SHA.findall((ROOT / "scripts" / "install.sh").read_text()), [])
        wf = ROOT / ".github" / "workflows" / "ci.yml"
        if wf.exists():
            self.assertEqual(set(SHA.findall(wf.read_text())), pin, "CI builds a different WLED commit than the scripts")

    def test_workflow_env_exists(self):
        wf = ROOT / ".github" / "workflows" / "ci.yml"
        if wf.exists():
            self.assertIn("ctl_esp32c6_8MB", wf.read_text())

    def test_scripts_parse(self):
        for sh in sorted((ROOT / "scripts").glob("*.sh")) + [ROOT / "tests" / "run.sh"]:
            r = subprocess.run(["bash", "-n", str(sh)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, f"{sh.name}: {r.stderr}")

    @unittest.skipUnless(shutil.which("shellcheck"), "shellcheck not installed")
    def test_scripts_shellcheck(self):
        scripts = [str(p) for p in sorted((ROOT / "scripts").glob("*.sh"))] + [str(ROOT / "tests" / "run.sh")]
        r = subprocess.run(["shellcheck", "-S", "warning"] + scripts, capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)


if __name__ == "__main__":
    unittest.main()
