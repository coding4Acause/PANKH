## Visualization Setup for Real-Time Plotting

To enable real-time wake visualization and dynamic lift coefficient plotting, the solver uses **Gnuplot**. Depending on your operating system, you will need to install Gnuplot along with a compatible **graphical terminal** (e.g., `x11`, `qt`, or `wxt`). You must also specify the desired terminal in your `input.json` file.

---

###  Step 1: Install Gnuplot

<details>
<summary><strong>Linux / Ubuntu</strong></summary>

```bash
sudo apt update
sudo apt install gnuplot
```

 **Test Installation:**

```bash
gnuplot
```
You should see the Gnuplot prompt:
```bash
gnuplot>
```
Type `exit` or press `Ctrl+D` to quit.
</details>

<details>
<summary><strong>macOS</strong></summary>

Install Gnuplot via **Homebrew**:

```bash
brew install gnuplot
```

 **Test Installation:**

```bash
gnuplot
```
You should see the Gnuplot prompt. Use `exit` to quit.
</details>

<details>
<summary><strong>Windows</strong></summary>

1. Download the Gnuplot installer from: [Gnuplot for Windows](http://www.gnuplot.info/download.html)
2. Run the installer and follow the instructions.
3. Make sure to **check the box** for installing with `wxt` terminal support.

 **Test Installation:**

Open **Command Prompt** and type:
```cmd
gnuplot
```
You should enter the Gnuplot terminal.
</details>

---

###  Step 2: Install Graphical Terminal Support

To render plots in a GUI window, Gnuplot needs a terminal driver.

<details>
<summary><strong>Linux: Install <code>X11</code> Support</strong></summary>

```bash
sudo apt install x11-apps
```

 **Verify Installation:**

```bash
xeyes  # Opens a small window with moving eyes (test for X11)
```

Also test in Gnuplot:

```bash
gnuplot
set terminal x11
```

If this fails, try installing with:

```bash
sudo apt install gnuplot-x11
```
</details>

<details>
<summary><strong>macOS: Use <code>qt</code> Terminal</strong></summary>

Gnuplot installed via Homebrew includes `qt` support by default.

 **Verify Terminal:**

```bash
gnuplot
set terminal qt
```

If this works without error, you're good to go.
</details>

<details>
<summary><strong>Windows: Use <code>wxt</code> Terminal</strong></summary>

The Windows version of Gnuplot typically supports `wxt` by default.

 **Verify Terminal:**

```cmd
gnuplot
set terminal wxt
```

If no error occurs, you're all set.
</details>

---

###  Step 3: Set Plotting Terminal in `input.json`

Specify the appropriate Gnuplot terminal in the `simulation` section of your `input.json` file:

```json
"simulation": {
  "wake": 0,
  "tolerance": 1e-6,
  "epsilon": 1e-8,
  "ncycles": 1,
  "nsteps": 40,
  "z": 200,
  "gnuplot_terminal": "x11"   // Change to "qt" or "wxt" based on your OS
}
```

✅ Supported terminals:
- `"x11"` for Linux (Ubuntu)
- `"qt"` for macOS
- `"wxt"` for Windows

The solver reads this value at runtime and automatically sets the terminal for real-time plotting, ensuring compatibility across all supported platforms.

