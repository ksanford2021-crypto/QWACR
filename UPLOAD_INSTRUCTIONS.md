# How to Upload Your qwacr_ws Workspace

The repository is now ready to receive your workspace files! Follow these steps to upload your qwacr_ws workspace from VS Code.

## Method 1: Using Git Commands (Recommended)

1. **Clone this repository** (if you haven't already):
   ```bash
   git clone https://github.com/ksanford2021-crypto/QWACR.git
   cd QWACR
   ```

2. **Copy your workspace files**:
   Copy the contents of your local `qwacr_ws/src/` directory into this repository's `qwacr_ws/src/` directory:
   ```bash
   # From your workspace location
   cp -r /path/to/your/qwacr_ws/src/* /path/to/QWACR/qwacr_ws/src/
   ```

3. **Add and commit your files**:
   ```bash
   cd /path/to/QWACR
   git add qwacr_ws/src/
   git commit -m "Add QWACR ROS packages"
   git push origin main
   ```

## Method 2: Using VS Code

1. **Open the repository in VS Code**:
   - File → Open Folder → Select the cloned QWACR directory

2. **Copy your packages**:
   - Copy your ROS packages from your local workspace into `qwacr_ws/src/`
   - VS Code will detect the new files

3. **Commit and push**:
   - Use the Source Control panel (Ctrl+Shift+G)
   - Stage all changes
   - Write a commit message
   - Click "Commit" then "Push"

## What Gets Committed

The `.gitignore` file is configured to:
- ✅ **Include**: Source code in `qwacr_ws/src/`
- ❌ **Exclude**: Build artifacts (`build/`, `devel/`, `install/`, `logs/`)
- ❌ **Exclude**: IDE files (`.vscode/`, `*.code-workspace`)
- ❌ **Exclude**: Temporary files (`*.pyc`, `*.log`, etc.)

This ensures you only commit your source code and not build artifacts or temporary files.

## Verify Your Upload

After pushing, verify your files are in the repository:
- Go to https://github.com/ksanford2021-crypto/QWACR
- Navigate to `qwacr_ws/src/`
- You should see your ROS packages

## Need Help?

If you encounter any issues:
1. Check that you're in the correct directory
2. Run `git status` to see what files will be committed
3. Make sure your packages are in `qwacr_ws/src/` directory
4. Ensure build artifacts are not being committed (they should be excluded by .gitignore)
