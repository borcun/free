var Hello = /** @class */ (function () {
    function Hello() {
    }
    Hello.prototype.WriteHelloWorld = function () {
        alert("Hello World");
    };
    return Hello;
}());
var hello = new Hello();
hello.WriteHelloWorld();
