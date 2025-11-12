document.querySelectorAll("tr").forEach(row => {
  const radios = row.querySelectorAll('input[type="radio"]');
  if (radios.length === 5) {
    radios[0].checked = true;
  }
});
