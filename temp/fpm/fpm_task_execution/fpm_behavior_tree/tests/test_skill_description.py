# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

"""Testing the skills.yaml existence and structure.
"""
from pathlib import Path
from typing import List, Literal, Optional, Union

import yaml
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    ValidationError,
    model_validator,
)


class SkillInput(BaseModel):
    """Data model for the skill inputs."""

    model_config = ConfigDict(extra="forbid")

    name: str
    type: Literal["string", "int", "float", "bool"]
    range: Optional[List[Union[int, float]]] = Field(
        None, min_length=2, max_length=2
    )
    description: str
    required: Optional[bool] = None
    default: Optional[Union[bool, float, int, str]] = None

    @model_validator(mode="after")
    def default_validator(self) -> "SkillInput":
        """Input validator."""
        if self.type == "string":
            if not isinstance(self.default, str):
                raise ValueError("Default value must be a string.")
            return self
        if self.type == "int":
            if not isinstance(self.default, int):
                raise ValueError("Default value must be an int.")
            return self
        if self.type == "float":
            if not isinstance(self.default, float):
                raise ValueError("Default value must be a float.")
            return self
        if self.type == "bool":
            if not isinstance(self.default, bool):
                raise ValueError("Default value must be a bool.")
            return self
        raise ValueError("Invalid type of input.")


class SkillDescription(BaseModel):
    """Data model for the skill description."""

    model_config = ConfigDict(extra="forbid")

    xml: str
    description: str
    inputs: List[SkillInput]
    released: bool
    test: Optional[bool] = None


def test_skills_yaml():
    """Test the skills.yaml file existence."""
    # Get the skills.yaml file path
    skills_description_path = Path.joinpath(
        Path(__file__).parents[1], "bt_xml", "skills.yaml"
    )

    # Check if the file exists
    assert skills_description_path.exists()

    # Check if the file is not empty
    assert skills_description_path.stat().st_size != 0

    # Check if the file is a valid yaml file
    with open(
        skills_description_path, "r", encoding="utf-8"
    ) as skills_description_file:
        try:
            yaml.safe_load(skills_description_file)
        except yaml.YAMLError as exc:
            assert (
                False
            ), f"The skills.yaml file is not a valid yaml file: {exc}"


def test_skills_yaml_structure():
    """Test the skills.yaml file structure."""

    # Get the skills.yaml file path
    skills_folder = Path.joinpath(Path(__file__).parents[1], "bt_xml")
    skills_description_path = Path.joinpath(skills_folder, "skills.yaml")

    with open(
        skills_description_path, "r", encoding="utf-8"
    ) as skills_description_file:
        try:
            skills = yaml.safe_load(skills_description_file)
        except yaml.YAMLError as exc:
            assert (
                False
            ), f"The skills.yaml file is not a valid yaml file: {exc}"

    # Get all xml files in the folder
    xml_files_paths = list(skills_folder.glob("*.xml"))
    xml_files = [xml_file_path.name for xml_file_path in xml_files_paths]

    # Check are all xml files in the skills.yaml file, and print the missing
    # ones
    skills_xmls = [skill["xml"] for skill in skills.values()]
    missing_xmls = [
        xml_file for xml_file in xml_files if xml_file not in skills_xmls
    ]
    assert (
        len(missing_xmls) == 0
    ), f"The following xml files are not described in the skills.yaml file\
    : {missing_xmls}"

    # Validate skills description
    for skill in skills.values():
        try:
            SkillDescription(**skill)
        except ValidationError as exc:
            assert (
                False
            ), f"The skill {skill['xml']} description is not valid: {exc}"
