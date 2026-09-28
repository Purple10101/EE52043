# Course files

Place the contents of `Applied_Robotics_Files` (github.bath.ac.uk/zr202/Applied_Robotics_Files) here.

Since that repo lives on the Bath enterprise GitHub instance and isn't accessible from this
VS Code session's GitHub account, get the files in one of these ways:

1. **Download as zip** — on the repo page, "Code" -> "Download ZIP", then extract the contents
   into this folder.
2. **Clone directly**, once you're authenticated to `github.bath.ac.uk` (e.g. via a personal
   access token from that instance):

   ```
   git clone https://github.bath.ac.uk/zr202/Applied_Robotics_Files.git course_files_src
   ```

   then move the files you need in here and delete `course_files_src`.

These files are treated as reference/course material, not code you're authoring, so they're
excluded from version control by default (see `.gitignore`). If you want them tracked in this
repo, remove the `course_files/` exclusion in `.gitignore`.
