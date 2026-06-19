# GitHub Upload Steps

## Option A: GitHub Desktop / Web

1. Create a new public repository on GitHub.
2. Name suggestion: `RawanFourMotionPlugin`.
3. Upload all files from this folder.
4. Commit with a message such as:

```text
Final N8RO four motion character animation plugin
```

5. Copy the public repository link and submit it with the demo video and written description.

## Option B: Command line

From inside this project folder:

```bash
git init
git add .
git commit -m "Final N8RO four motion character animation plugin"
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/RawanFourMotionPlugin.git
git push -u origin main
```

Replace `YOUR_USERNAME` with your GitHub username.

## GitHub CLI option

If GitHub CLI is installed:

```bash
gh repo create RawanFourMotionPlugin --public --source=. --remote=origin --push
```
