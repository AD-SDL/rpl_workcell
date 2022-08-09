import yaml
import json
from pathlib import Path
from uuid import UUID, uuid4
from typing import Dict, Union, TypeVar, Type, Optional, List


from pydantic import BaseModel as _BaseModel
from pydantic import Field

_T = TypeVar("_T")

PathLike = Union[str, Path]


class BaseModel(_BaseModel):
    """Allows any sub-class to inherit methods allowing for programatic description of protocols
    Can load a yaml into a class and write a class into a yaml file.
    """

    def write_yaml(self, cfg_path: PathLike) -> None:
        """Allows programatic creation of ot2util objects and saving them into yaml.
        Parameters
        ----------
        cfg_path : PathLike
            Path to dump the yaml file.
        """
        with open(cfg_path, mode="w") as fp:
            yaml.dump(json.loads(self.json()), fp, indent=4, sort_keys=False)

    @classmethod
    def from_yaml(cls: Type[_T], filename: PathLike) -> _T:
        """Allows loading of yaml into ot2util objects.
        Parameters
        ----------
        filename: PathLike
            Path to yaml file location.
        """
        with open(filename) as fp:
            raw_data = yaml.safe_load(fp)
        return cls(**raw_data)  # type: ignore[call-arg]


class Tag(BaseModel):
    type: str
    id: str  # not quite sure what this will be


class Module(BaseModel):
    name: str
    type: str
    config: Dict  # contains ip and port
    positions: Optional[dict]
    tag: Optional[Tag]
    id: UUID = Field(default_factory=uuid4)


class SimpleModule(BaseModel):
    name: str
    type: str
    id: Optional[Union[UUID, str]]
    # what else? Equipemnt it needs?


class Command(BaseModel):
    name: str
    instruction: str
    args: Dict
    checks: Optional[str]
    comment: Optional[str]


class Step(BaseModel):
    name: str
    module: str
    commands: List[Command]
    requirements: Optional[Dict]
    dependencies: Optional[Union[str, UUID]]
    priority: Optional[int]
    id: UUID = Field(default_factory=uuid4)
    comment: Optional[str]


class Metadata(BaseModel):
    name: Optional[str]
    author: Optional[str]
    info: Optional[str]
    version: float = 0.1


class Workflow(BaseModel):
    workcell: Union[str, Path]
    modules: List[SimpleModule]
    flowdef: List[Step]
    metadata: Metadata


class WorkCell(BaseModel):
    modules: List[Module]
