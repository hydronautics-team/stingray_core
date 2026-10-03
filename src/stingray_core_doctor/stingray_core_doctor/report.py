from stingray_core_doctor.types import CheckResult, CheckStatus


class Reporter:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[31m"

    def __init__(self):
        self._all_results: list[CheckResult] = []

    def format_status(self, status: CheckStatus) -> str:   

        if status == CheckStatus.OK:
            return f"{self.GREEN} [ OK ] {self.RESET}"
        elif status == CheckStatus.WARN:
            return f"{self.YELLOW} [WARN] {self.RESET}"
        else:
            return f"{self.RED} [FAIL] {self.RESET}"

    def print_header(self):
        
        print(f"{self.BOLD}Stingray Core Doctor{self.RESET}")    
        print("="*15)

    def print_category(self, category_name: str, results: list[CheckResult]):

        self._all_results.extend(results)

        print(f"\n{category_name}")
        for r in results:
            status = self.format_status(r.status)
            print(f"{status}{r.name}")
            if r.message:
                print(f"        Reason: {r.message}")
            if r.hint:
                print(f"        Hint:   {r.hint}")

    def calculate_exit_code(self) -> int:
        
        has_fail = False
        has_warn = False

        for r in self._all_results:
            if r.status == CheckStatus.FAIL:
                has_fail = True
            elif r.status == CheckStatus.WARN:
                has_warn = True

        if has_fail:
            return int(CheckStatus.FAIL)
        if has_warn:
            return int(CheckStatus.WARN)
        return int(CheckStatus.OK)

    def print_summary(self) -> int:

        exit_code = self.calculate_exit_code()

        if exit_code == int(CheckStatus.FAIL):
            print(f"\nResults: {self.RED}FAIL{self.RESET}")
        elif exit_code == int(CheckStatus.WARN):
            print(f"\nResults: {self.YELLOW}WARN{self.RESET}")
        else:
            print(f"\nResults: {self.GREEN}OK{self.RESET}")
        
        return exit_code
