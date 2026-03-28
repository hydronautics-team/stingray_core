import os
from pathlib import Path
from logging import Logger
from ament_index_python.packages import get_package_share_directory
import yaml
from rclpy.parameter import Parameter

PACKAGE_NAME = "stingray_core_control"
PKG_DIR_NAME = "stingray_core_control"  # каталог пакета в src

def load_existing_yaml(path: Path) -> dict:
    if path.exists():
        try:
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except Exception:
            return {}
    return {}

def get_src_param_path(config_name: str) -> Path:
    """Путь к YAML в исходниках (для git)"""
    ws_env = os.environ.get("STINGRAY_WS")
    if ws_env:
        return Path(ws_env) / "src" / PKG_DIR_NAME / "params" / f"{config_name}.yaml"

    # Ищем package.xml вверх от текущего файла
    cur = Path(__file__).resolve()
    for parent in [cur] + list(cur.parents):
        if (parent / "package.xml").exists():
            return parent / "params" / f"{config_name}.yaml"

    # fallback на стандартный путь контейнера
    return Path("/stingray_core/src") / PKG_DIR_NAME / "params" / f"{config_name}.yaml"

def get_runtime_param_path(config_name: str) -> Path:
    """Путь для записи runtime-параметров (install/share или ~/.config)"""
    try:
        share_dir = get_package_share_directory(PACKAGE_NAME)
        runtime_path = Path(share_dir) / "params" / f"{config_name}.yaml"
        return runtime_path
    except Exception:
        # fallback в домашний каталог
        return Path.home() / ".config" / PACKAGE_NAME / f"{config_name}.yaml"

def save_params(self, param_list: list[Parameter], config_name: str):
    """
    Сохраняет параметры одновременно:
      - в исходники (src/.../params) — для git
      - в runtime путь (install/share/... или ~/.config)
    """

    # --- Собираем snapshot параметров ---
    current_params = {}

    # Читаем то, что уже лежит в YAML, чтобы при частичном обновлении
    # не потерять старые ключи (merge, а не полная замена).
    src_file = get_src_param_path(config_name)
    existing_yaml = load_existing_yaml(src_file)
    node_name = self.get_name()

    # Пояснение к проверке:
    # 1) existing_yaml должен быть dict
    # 2) в нём должен быть раздел текущей ноды (node_name)
    # 3) внутри должен быть ключ 'ros__parameters'
    # 4) и это должен быть dict параметров (иначе update небезопасен)
    if (
        isinstance(existing_yaml, dict)
        and node_name in existing_yaml
        and 'ros__parameters' in existing_yaml[node_name]
        and isinstance(existing_yaml[node_name]['ros__parameters'], dict)
    ):
        current_params.update(existing_yaml[node_name]['ros__parameters'])

    try:
        for name, p in self._parameters.items():
            if config_name == "thruster_matrix":
                for thr in self.thrusters:
                    for a in self.axes:
                        if name == f"{thr}_{a}":
                            current_params[name] = p.value
            elif config_name == "controllers":
                for axis in self.axes:
                    for key in self.param_keys:
                        if name == f'controllers.{axis}.{key}':
                            current_params[name] = p.value
            elif config_name == "stingray_core_control_node":
                # Для node-конфига merge уже выполнен выше,
                # здесь просто оставляем существующее поведение.
                pass

    except Exception:
        current_params = {p.name: p.value for p in param_list}  # хотя бы param_list

    # --- Обновляем ключи из param_list ---
    for p in param_list:
        current_params[p.name] = p.value

    # --- Формируем структуру для YAML ---
    out = {self.get_name(): {'ros__parameters': current_params}}

    # --- Сохраняем в исходники ---
    src_file = get_src_param_path(config_name)
    try:
        src_file.parent.mkdir(parents=True, exist_ok=True)
        with open(src_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(out, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
        self.get_logger().info(f"Параметры сохранены в исходники: {src_file}")
    except Exception as e:
        self.get_logger().warning(f"Не удалось записать в src: {src_file}, ошибка: {e}")

    # --- Сохраняем в runtime путь ---
    runtime_file = get_runtime_param_path(config_name)
    try:
        runtime_file.parent.mkdir(parents=True, exist_ok=True)
        with open(runtime_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(out, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
        self.get_logger().info(f"Параметры сохранены в runtime путь: {runtime_file}")
    except Exception as e:
        self.get_logger().error(f"Не удалось записать runtime YAML: {runtime_file}, ошибка: {e}")
