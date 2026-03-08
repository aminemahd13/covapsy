# CubeIDE Project Placeholder

Create or import the STM32CubeIDE project in this folder.

Recommended structure after generation:

- `cubeide_project/<your_project_name>/Core/Inc`
- `cubeide_project/<your_project_name>/Core/Src`
- `cubeide_project/<your_project_name>/.project`
- `cubeide_project/<your_project_name>/.cproject`

Do not commit heavy build outputs:

- `Debug/`
- `Release/`
- `.settings/`

Use `../src` as the canonical firmware logic source and copy/import into `Core`.
