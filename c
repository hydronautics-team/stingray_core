#!/usr/bin/env python3

import subprocess
import click
import sys
import os
from enum import Enum, auto

# Параметры
PROJECT_NAME = "stingray-core"
IMAGE_NAME = "hydronautics/" + PROJECT_NAME + ":dev"
CONTAINER_NAME = PROJECT_NAME + "-dev"


def run_cmd(cmd, capture_output=True):
    """Выполняет команду и возвращает результат."""
    if capture_output:
        return subprocess.run(cmd, capture_output=True, text=True)
    return subprocess.run(cmd)


def exit(msg: str):
    """Выход из программы с сообщением."""
    click.echo(msg)
    sys.exit(1)


class ContainerStatus(Enum):
    """Статусы контейнера."""

    STOP = auto()
    UP = auto()
    NONE = auto()


def check_container_status() -> ContainerStatus:
    """Проверка статуса контейнера."""
    result = run_cmd(
        [
            "docker",
            "ps",
            "-a",
            "--filter",
            f"name={CONTAINER_NAME}",
            "--format",
            "{{.Status}}",
        ],
    )
    if not result.stdout:
        return ContainerStatus.NONE
    if "Up" in result.stdout:
        return ContainerStatus.UP
    return ContainerStatus.STOP


def remove_image() -> bool:
    """Удаление старого образа."""
    click.echo(f"Проверяем наличие старого образа '{IMAGE_NAME}'...")
    result = run_cmd(["docker", "images", "-q", IMAGE_NAME], capture_output=True)

    if result.stdout.strip():
        click.echo(f"Удаляем старый образ '{IMAGE_NAME}'...")
        del_result = run_cmd(["docker", "rmi", "-f", IMAGE_NAME], capture_output=True)
        if del_result.returncode == 0:
            click.echo(f"✅ Старый образ '{IMAGE_NAME}' удалён.")
        else:
            click.echo(
                f"⚠️ Не удалось удалить старый образ: {del_result.stderr.strip()}"
            )
            return False
    else:
        click.echo(f"✅ Старый образ '{IMAGE_NAME}' не найден. Продолжаем...")

    return True


class AliasedGroup(click.Group):
    """Класс для создания alias для команд."""

    def get_command(self, ctx, cmd_name):
        rv = click.Group.get_command(self, ctx, cmd_name)
        if rv is not None:
            return rv

        aliases = {
            "b": "build",
            "u": "up",
            "a": "attach",
            "d": "delete",
            "s": "status",
        }
        if cmd_name in aliases:
            return click.Group.get_command(self, ctx, aliases[cmd_name])
        return None


@click.group(cls=AliasedGroup)
def cli():
    """CLI для управления Docker-контейнером."""
    pass


@cli.command()
@click.option(
    "--user",
    default=os.environ.get("USER"),
    help="Имя пользователя в контейнере.",
)
@click.option("--no-cache", is_flag=True, help="Сборка без использования кэша.")
def build(user: str | None, no_cache: bool):
    """Сборка Docker-образа."""
    if check_container_status() != ContainerStatus.NONE:
        exit(
            f"❌ Контейнер на базе образа '{IMAGE_NAME}' уже существует. Остановите и удалите его командой 'delete/d'."
        )

    if user is None:
        exit(
            "❌ Невозможно получить имя пользователя из переменных окружения. Объявите его в переменных или укажите вручную."
        )

    if not remove_image():
        sys.exit(1)
    cmd = ["docker", "build", "-t", IMAGE_NAME, "--build-arg", f"NEW_USER={user}", "."]

    if no_cache:
        cmd.insert(-1, "--no-cache")
        click.echo("Сборка без использования кэша.")

    click.echo(f"Сборка образа '{IMAGE_NAME}'.")
    result = run_cmd(cmd, capture_output=False)

    if result.returncode == 0:
        click.echo(f"✅ Образ '{IMAGE_NAME}' успешно собран.")
    else:
        exit(f"❌ Ошибка при сборке образа '{IMAGE_NAME}'.")


@cli.command()
def up():
    """Запуск контейнера из образа."""
    status = check_container_status()

    if status == ContainerStatus.UP:
        click.echo(f"✅ Контейнер {CONTAINER_NAME} уже запущен.")
    elif status == ContainerStatus.STOP:
        click.echo(f"Контейнер {CONTAINER_NAME} остановлен. Запускаем...")
        run_cmd(["docker", "start", CONTAINER_NAME])
        click.echo(f"✅ Контейнер {CONTAINER_NAME} запущен.")
    elif status == ContainerStatus.NONE:
        click.echo(f"Контейнер {CONTAINER_NAME} не найден. Запускаем новый...")
        run_cmd(
            [
                "docker",
                "run",
                "-d",
                "-v",
                f"{os.getcwd()}:/stingray-core",
                "--name",
                CONTAINER_NAME,
                "-it",
                IMAGE_NAME,
                "/bin/bash",
            ]
        )
        click.echo(f"✅ Контейнер {CONTAINER_NAME} запущен.")
    else:
        assert False, "unknown container status"


@cli.command()
def attach():
    """Подключение к контейнеру."""
    result = run_cmd(
        [
            "docker",
            "ps",
            "--filter",
            f"name={CONTAINER_NAME}",
            "--format",
            "{{.Names}}",
        ],
    )
    if CONTAINER_NAME in result.stdout:
        click.echo(f"Подключение к контейнеру {CONTAINER_NAME}...")
        run_cmd(["docker", "exec", "-it", CONTAINER_NAME, "bash"], capture_output=False)
    else:
        exit(f"❌ Контейнер {CONTAINER_NAME} не запущен.")


@cli.command()
def delete():
    """Остановка и удаление контейнеров."""
    result = run_cmd(
        [
            "docker",
            "ps",
            "-a",
            "--filter",
            f"name={CONTAINER_NAME}",
            "--format",
            "{{.Names}}",
        ],
        capture_output=True,
    )
    if CONTAINER_NAME in result.stdout:
        click.echo(f"Останавливаем контейнер {CONTAINER_NAME}...")
        run_cmd(["docker", "stop", CONTAINER_NAME])
        click.echo(f"Удаляем контейнер {CONTAINER_NAME}...")
        run_cmd(["docker", "rm", CONTAINER_NAME])
        click.echo(f"✅ Контейнер {CONTAINER_NAME} остановлен и удалён.")
    else:
        click.echo(f"⚠️ Контейнер {CONTAINER_NAME} не найден.")


@cli.command()
def status():
    """Проверка статуса контейнера."""
    result = run_cmd(
        [
            "docker",
            "ps",
            "-a",
            "--filter",
            f"name={CONTAINER_NAME}",
            "--format",
            "{{.Status}}",
        ],
    )

    if result.stdout:
        click.echo(f"📦 Контейнер: {CONTAINER_NAME}")
        click.echo(f"✅ Статус: {result.stdout.strip()}")
    else:
        click.echo(f"⚠️ Контейнер {CONTAINER_NAME} не существует.")


if __name__ == "__main__":
    cli()
